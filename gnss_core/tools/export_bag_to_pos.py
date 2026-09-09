#!/usr/bin/env python3
"""export_bag_to_pos.py —— 把 rtk-monitor 既有录包中的历元导出为标准 RTKLIB .pos(spec §9.1 轮 1 数据来源)。

用法:
  export_bag_to_pos.py --db data/2026-09-03.db --src can   --out /data/gnss/20260903/can.pos
  export_bag_to_pos.py --db data/2026-09-03.db --src gpchc --out /data/gnss/20260903/gpchc.pos
  export_bag_to_pos.py --db data/2026-09-03.db --src rtkrcv --out /data/gnss/20260903/rtkrcv.pos
  export_bag_to_pos.py --db data/2026-09-03.db --src can --out can.pos --t0 1757300000 --t1 1757303600
  export_bag_to_pos.py --pos existing.pos --src rtkrcv --out out.pos     # 已是 .pos:只做时间系统/列规范化

输出格式与 gnss_core::read_pos 的解析一致(Task 10):
  % (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=UTC)
  YYYY/MM/DD HH:MM:SS.sss lat lon height Q ns sdn sde sdu sdne sdeu sdun age ratio
时间列固定写 UTC(头部 time=UTC),因为 rtk-monitor 的历元时间来自 ROS/系统时钟(UTC unix 秒),
不是 GPST;这样 calibrate_sigma_scale 与 rnx2rtkp 输出的 GPST .pos 按 0.1 s 容差配对不会落空。

rtk-monitor 的 SQLite schema(已核实:rtk_monitor/storage/epochs.py):
  epochs(id, t REAL unix秒(UTC), src TEXT ∈{rtkrcv,gpchc,can}, q INTEGER, sats, age,
         lat, lon, alt, sde, sdn, sdu, ratio, heading, speed, sats_json)
  - `q` 语义按 src:rtkrcv 是 RTKLIB Q(1 fix 2 float 4 dgps 5 single);
    gpchc/can 是 CGI-610 satellite_status:4/8→fix 5/9→float 2/7→dgps 1/6/3→single 0→无解。
  - 无解(Q=0)默认跳过,`--keep-nofix` 保留并写 Q=0(read_pos 映射为 NONE)。
  - 注意 DB 的 σ 列顺序是 sde sdn sdu,.pos 是 sdn sde sdu;ratio/age/sats 为 NULL 写 0。
  - `--t0/--t1`(unix 秒,闭区间)截取时间段。
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
# 输入:SQLite(rtk-monitor epochs 表)
# ---------------------------------------------------------------------------
# CGI-610 satellite_status → RTKLIB Q。未知码按无解处理(Q=0)。
_CGI610_STATUS_TO_Q = {
    4: 1, 8: 1,        # RTK fixed(含/不含航向)
    5: 2, 9: 2,        # RTK float
    2: 4, 7: 4,        # DGPS
    1: 5, 6: 5, 3: 5,  # single(3:2D 单点,当 single)
    0: 0,              # 无解
}


def cgi610_status_to_q(status) -> int:
    if status is None:
        return 0
    return _CGI610_STATUS_TO_Q.get(int(status), 0)


def rtkrcv_q_to_q(q) -> int:
    """RTKLIB Q 原样透传;NULL/0/其它(3=SBAS 等 gnss_core 视为 NONE 的值)写 0。"""
    if q is None:
        return 0
    q = int(q)
    return q if q in (1, 2, 4, 5) else 0


def _f(v, default=0.0) -> float:
    return default if v is None else float(v)


def load_epochs_from_db(db_path: str, src: str, t0: float = None, t1: float = None,
                        keep_nofix: bool = False) -> List[Epoch]:
    """从 rtk-monitor 的 SQLite 录包读取某一源的历元(t 视为 UTC unix 秒)。

    - 缺 lat/lon/alt 的行跳过;Q=0(无解)默认跳过,keep_nofix=True 时保留;
    - sats/age/ratio/σ 为 NULL 写 0;t0/t1 闭区间过滤(None = 不限)。
    表不存在抛 RuntimeError(并列出 DB 内实际表名)。
    """
    if src not in SOURCES:
        raise ValueError(f"unknown src {src!r}; expected one of {SOURCES}")
    to_q = rtkrcv_q_to_q if src == "rtkrcv" else cgi610_status_to_q

    sql = ("SELECT t, q, sats, age, lat, lon, alt, sde, sdn, sdu, ratio FROM epochs WHERE src=?")
    params: list = [src]
    if t0 is not None:
        sql += " AND t>=?"
        params.append(float(t0))
    if t1 is not None:
        sql += " AND t<=?"
        params.append(float(t1))
    sql += " ORDER BY t"

    conn = sqlite3.connect(db_path)
    try:
        try:
            rows = conn.execute(sql, params).fetchall()
        except sqlite3.OperationalError as e:
            tables = [r[0] for r in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")]
            raise RuntimeError(f"{db_path}: cannot read epochs table ({e}); tables present: {tables}") from e
    finally:
        conn.close()

    out: List[Epoch] = []
    for t, q, sats, age, lat, lon, alt, sde, sdn, sdu, ratio in rows:
        if t is None or lat is None or lon is None or alt is None:
            continue
        rq = to_q(q)
        if rq == 0 and not keep_nofix:
            continue
        out.append(Epoch(float(t), float(lat), float(lon), float(alt), rq, int(_f(sats)),
                         _f(sdn), _f(sde), _f(sdu), _f(age), _f(ratio)))
    return out


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
    ap.add_argument("--t0", type=float, default=None, help="--db:起始时间(UTC unix 秒,含)")
    ap.add_argument("--t1", type=float, default=None, help="--db:结束时间(UTC unix 秒,含)")
    ap.add_argument("--keep-nofix", action="store_true",
                    help="--db:保留无解历元并写 Q=0(默认跳过)")
    a = ap.parse_args(argv)

    if a.db:
        epochs = load_epochs_from_db(a.db, a.src, a.t0, a.t1, a.keep_nofix)
    else:
        epochs = load_epochs_from_pos(a.pos, assume_gpst=(a.pos_time == "GPST"))
    n = write_pos(a.out, epochs, a.src)
    print(f"wrote {n} epochs -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
