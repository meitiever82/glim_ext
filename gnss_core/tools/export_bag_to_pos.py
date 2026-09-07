#!/usr/bin/env python3
"""export_bag_to_pos.py —— 把 rtk-monitor 既有录包中的历元导出为标准 RTKLIB .pos(spec §9.1 轮 1 数据来源)。

用法:
  export_bag_to_pos.py --db data/2026-09-03.db --src can   --out /data/gnss/20260903/can.pos
  export_bag_to_pos.py --db data/2026-09-03.db --src gpchc --out /data/gnss/20260903/gpchc.pos
  export_bag_to_pos.py --db data/2026-09-03.db --src rtkrcv --out /data/gnss/20260903/rtkrcv.pos
  export_bag_to_pos.py --pos existing.pos --src rtkrcv --out out.pos     # 已是 .pos:只做时间系统/列规范化

输出格式与 gnss_core::read_pos 的解析一致(Task 10):
  % (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=UTC)
  YYYY/MM/DD HH:MM:SS.sss lat lon height Q ns sdn sde sdu sdne sdeu sdun age ratio
时间列固定写 UTC(头部 time=UTC),因为 rtk-monitor 的历元时间来自 ROS/系统时钟(UTC unix 秒),
不是 GPST;这样 calibrate_sigma_scale 与 rnx2rtkp 输出的 GPST .pos 按 0.1 s 容差配对不会落空。

====================================================================================
TODO(现场):rtk-monitor 的 SQLite schema 在本仓库中不可见,下面的 `load_epochs_from_db`
只是一个占位实现,**字段名/表名全部需要按实际 schema 填写**,不要沿用这里的猜测:
  - 表名:rtk-monitor 存历元的表(PLAN 称 "epochs" 表,未核实)
  - 源区分列:can / gpchc / rtkrcv 是不同表还是同一表的 source 列?
  - 时间列:单位(s / ms / ns)与时间系统(ROS 时间=UTC unix?板卡 GPST?)
  - 质量列:各源的质量编码不同(CGI610 的 CAN/GPCHC 状态码 ≠ RTKLIB Q),需映射到 RTKLIB Q:
        FIXED→1  FLOAT→2  DGPS→4  SINGLE→5  其它→0
  - σ 列:是否有 sdn/sde/sdu?CAN 融合解可能只有一个 3D σ 或根本没有 → 写 0(read_pos/compare_by_quality
    会把 σ_h=0 的历元排除在比值统计之外,但仍计入 n 与 RMSE)
  - 卫星数 / 差分龄期 / ratio:缺失写 0
填好后用 tools/make_synthetic_pos.py + calibrate_sigma_scale 自检输出能被解析。
====================================================================================
"""
import argparse
import calendar
import math
import sqlite3
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List

SOURCES = ("can", "gpchc", "rtkrcv")
GPS_UTC_LEAP = 18


@dataclass
class Epoch:
    stamp_utc: float      # unix 秒(UTC)
    lat: float            # deg
    lon: float            # deg
    height: float         # m,椭球高
    q: int                # RTKLIB Q
    ns: int = 0
    sdn: float = 0.0
    sde: float = 0.0
    sdu: float = 0.0
    age: float = 0.0
    ratio: float = 0.0


# ---------------------------------------------------------------------------
# 输入:SQLite(TODO 见文件头)
# ---------------------------------------------------------------------------
def load_epochs_from_db(db_path: str, src: str) -> List[Epoch]:
    """从 rtk-monitor 的 SQLite 录包读取某一源的历元。

    TODO(现场):按实际 schema 实现。以下仅为骨架,故意不猜列名。
    实现步骤:
      1. `sqlite3 <db> .schema` 看表结构,确定历元表与 source 区分方式;
      2. 写 SELECT,把每行映射成 Epoch;
      3. 时间统一成 UTC unix 秒(若列是 GPST,减 GPS_UTC_LEAP;若是 ms/ns,除以 1e3/1e9);
      4. 质量映射到 RTKLIB Q(见文件头)。
    """
    conn = sqlite3.connect(db_path)
    try:
        tables = [r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")]
    finally:
        conn.close()
    raise NotImplementedError(
        "load_epochs_from_db: rtk-monitor schema not yet filled in. "
        f"Tables found in {db_path}: {tables}. See TODO at top of this file (src={src})."
    )


# ---------------------------------------------------------------------------
# 输入:已有 .pos(只做规范化,便于把非 RTKLIB 产出的 .pos 变体统一)
# ---------------------------------------------------------------------------
def load_epochs_from_pos(path: str, assume_gpst: bool = True) -> List[Epoch]:
    out: List[Epoch] = []
    time_is_gpst = assume_gpst
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            if line[0] == "%":
                if "time=GPST" in line:
                    time_is_gpst = True
                elif "time=UTC" in line:
                    time_is_gpst = False
                else:
                    # RTKLIB 列名行 "%  GPST  latitude(deg) ..." / "%  UTC ...":第一个 token 即时间系统
                    tok = line[1:].split()
                    if tok and tok[0] == "GPST":
                        time_is_gpst = True
                    elif tok and tok[0] == "UTC":
                        time_is_gpst = False
                continue
            c = line.split()
            if len(c) < 10:
                continue
            y, mo, d = (int(x) for x in c[0].split("/"))
            hh, mm, ss = c[1].split(":")
            base = calendar.timegm((y, mo, d, int(hh), int(mm), 0))   # 按 UTC 日历,不受本机 TZ 影响
            stamp = base + float(ss)
            if time_is_gpst:
                stamp -= GPS_UTC_LEAP
            vals = [float(x) for x in c[2:]] + [0.0] * 13
            out.append(Epoch(stamp, vals[0], vals[1], vals[2], int(vals[3]), int(vals[4]),
                             vals[5], vals[6], vals[7], vals[11], vals[12]))
    return out


# ---------------------------------------------------------------------------
# 输出:标准 .pos(UTC 头)
# ---------------------------------------------------------------------------
def _fmt_time(stamp_utc: float) -> str:
    whole = math.floor(stamp_utc)
    ms = int(round((stamp_utc - whole) * 1000))
    if ms >= 1000:
        ms -= 1000
        whole += 1
    tm = time.gmtime(whole)
    return "%04d/%02d/%02d %02d:%02d:%02d.%03d" % (tm.tm_year, tm.tm_mon, tm.tm_mday,
                                                   tm.tm_hour, tm.tm_min, tm.tm_sec, ms)


def write_pos(path: str, epochs: Iterable[Epoch], src: str) -> int:
    n = 0
    with open(path, "w") as f:
        f.write(f"% program   : export_bag_to_pos.py (src={src})\n")
        f.write("% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=UTC)\n")
        f.write("%  UTC                   latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)"
                "   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")
        for e in sorted(epochs, key=lambda e: e.stamp_utc):
            f.write("%s %14.9f %14.9f %10.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n" % (
                _fmt_time(e.stamp_utc), e.lat, e.lon, e.height, e.q, e.ns,
                e.sdn, e.sde, e.sdu, 0.0, 0.0, 0.0, e.age, e.ratio))
            n += 1
    return n


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src_in = ap.add_mutually_exclusive_group(required=True)
    src_in.add_argument("--db", help="rtk-monitor SQLite 录包(data/*.db)")
    src_in.add_argument("--pos", help="已有 .pos,只做时间系统/列规范化")
    ap.add_argument("--src", choices=SOURCES, required=True, help="导出哪一路解")
    ap.add_argument("--out", required=True, help="输出 .pos 路径")
    ap.add_argument("--pos-time", choices=("GPST", "UTC"), default="GPST",
                    help="--pos 输入无头部标注时假定的时间系统(默认 GPST,与 RTKLIB 一致)")
    a = ap.parse_args(argv)

    if a.db:
        epochs = load_epochs_from_db(a.db, a.src)
    else:
        epochs = load_epochs_from_pos(a.pos, assume_gpst=(a.pos_time == "GPST"))
    n = write_pos(a.out, epochs, a.src)
    print(f"wrote {n} epochs -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
