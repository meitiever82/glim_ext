#!/usr/bin/env python3
"""field_eval.py —— 整链路回放一遍(run_A / run_B)的结果评估,输出 Markdown。

用法:
  field_eval.py --run <run_dir> --ref <seg>/gnss/rtk_check.pos --out <md> [--bag <run_dir>/bags/gnss_*]
                [--label A] [--pair-tol 0.1]

读取(全部只读):
  <run>/pos/*/can.pos、<run>/pos/*/rtkrcv.pos   pos_writer 输出(日期时间列,头部 time=GPST)
  <ref>                                           rnx2rtkp 输出(GPST 周 + 周内秒列;头部 "% ref pos")
  <run>/diag/*/events.log、<run>/diag/*/base.pos、<run>/diag/base_baseline
  <run>/rtkrcv/rtkrcv.conf                        只摘几条关键配置,便于对比修复前后
  录包(--bag,缺省自动找 <run>/bags/gnss_* 唯一目录):/gnss_cgi610/rtk_fix(航向、速度)、
  /gnss/diagnostics、/rtkrcv_node/diagnostics。读录包要先 source ROS 与 glim_ws/driver_ws overlay
  (反序列化 gnss_msgs);读不了时相应小节写明原因,航向退回用 can.pos 相邻历元的航迹向。

时间:内部统一用 UTC unix 秒。.pos 按头部判定时间系统(与 gnss_core read_pos 同规则:"time=GPST|UTC"
或列名行首词 GPST/UTC,都没有时按 GPST),GPST 固定减 18 s。gnss_core 的 read_pos(C++)与
export_bag_to_pos.load_epochs_from_pos(Python)都只认日期时间列,不认 rnx2rtkp 默认的"周 周内秒"
列,所以这里自己解析两种格式(见 tests/test_field_eval.py)。

符号约定:位置差 = 被评估源 − ref,在 ref 点的当地 ENU 下表示;车体系分解中"前"= 航向方向,
"右"= 航向顺时针 90°;航向北零顺时针(RtkFix.heading 同义)。
"""
import argparse
import bisect
import glob
import math
import os
import sys
from collections import Counter, OrderedDict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

GPS_EPOCH_UNIX = 315964800          # 1980-01-06 00:00:00
GPS_UTC_LEAP = 18
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

Q_NAMES = {1: "fix", 2: "float", 3: "sbas", 4: "dgps", 5: "single", 6: "ppp"}


# ---------------------------------------------------------------------------
# .pos 解析
# ---------------------------------------------------------------------------
@dataclass
class PosRecord:
    t: float          # UTC unix 秒
    lat: float
    lon: float
    height: float
    q: int
    ns: int
    sdn: float = 0.0
    sde: float = 0.0
    sdu: float = 0.0
    age: float = 0.0
    ratio: float = 0.0


@dataclass
class PosFile:
    path: str
    time_system: str                       # "GPST" | "UTC"
    ref_llh: Optional[Tuple[float, float, float]]
    records: List[PosRecord]


def _calendar_to_unix(date: str, tod: str) -> Optional[float]:
    import calendar
    try:
        y, mo, d = (int(x) for x in date.split("/"))
        hh, mm, ss = tod.split(":")
        return calendar.timegm((y, mo, d, int(hh), int(mm), 0)) + float(ss)   # UTC 日历,不受本机 TZ 影响
    except (ValueError, TypeError):
        return None


def parse_pos_data_line(line: str, time_system: str) -> Optional[PosRecord]:
    """一行 .pos 数据 → PosRecord(t 为 UTC unix 秒);不是数据行或列数不足返回 None。

    时间列两种:"YYYY/MM/DD HH:MM:SS.sss"(pos_writer、rnx2rtkp -t)或 "WEEK TOW"(rnx2rtkp 默认)。
    其后固定 lat lon height Q ns sdn sde sdu,可选 sdne sdeu sdun age ratio。
    """
    c = line.split()
    if len(c) < 10 or c[0].startswith("%"):
        return None
    if "/" in c[0]:
        t = _calendar_to_unix(c[0], c[1])
        if t is None:
            return None
    else:
        try:
            week = int(c[0])
            tow = float(c[1])
        except ValueError:
            return None
        t = GPS_EPOCH_UNIX + week * 604800 + tow
    try:
        v = [float(x) for x in c[2:]]
    except ValueError:
        return None
    if time_system == "GPST":
        t -= GPS_UTC_LEAP
    opt = v[8:13] + [0.0] * (5 - len(v[8:13]))
    return PosRecord(t=t, lat=v[0], lon=v[1], height=v[2], q=int(v[3]), ns=int(v[4]),
                     sdn=v[5], sde=v[6], sdu=v[7], age=opt[3], ratio=opt[4])


def read_pos(path: str) -> PosFile:
    ts = "GPST"
    ref = None
    recs: List[PosRecord] = []
    with open(path, errors="replace") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("%"):
                body = s[1:].strip()
                if "x-ecef" in body or "e-baseline" in body:
                    raise ValueError(f"{path}: 只支持经纬高(llh)格式的 .pos,这是 xyz/enu 格式")
                if "time=GPST" in body:
                    ts = "GPST"
                elif "time=UTC" in body:
                    ts = "UTC"
                else:
                    tok = body.split()
                    if tok and tok[0] in ("GPST", "UTC"):
                        ts = tok[0]
                if body.startswith("ref pos"):
                    try:
                        vals = [float(x) for x in body.split(":", 1)[1].split()[:3]]
                        if len(vals) == 3 and abs(vals[0]) <= 90 and abs(vals[1]) <= 360 and abs(vals[2]) < 1e5:
                            ref = (vals[0], vals[1], vals[2])
                    except (ValueError, IndexError):
                        pass
                continue
            r = parse_pos_data_line(s, ts)
            if r is not None:
                recs.append(r)
    return PosFile(path=path, time_system=ts, ref_llh=ref, records=recs)


# ---------------------------------------------------------------------------
# 配对与统计
# ---------------------------------------------------------------------------
def pair_by_time(a: Sequence[float], b: Sequence[float], tol: float) -> List[Tuple[int, int]]:
    """按时间一对一配对:每个 a 找最近的 b(|dt| ≤ tol);多个 a 争同一个 b 时留最近的那个。
    返回 (a 原下标, b 原下标),按 a 下标升序。"""
    if not a or not b:
        return []
    order = sorted(range(len(b)), key=lambda i: b[i])
    bs = [b[i] for i in order]
    best: Dict[int, Tuple[float, int]] = {}
    for ia, ta in enumerate(a):
        k = bisect.bisect_left(bs, ta)
        cand = None
        for j in (k - 1, k):
            if 0 <= j < len(bs):
                dt = abs(bs[j] - ta)
                if cand is None or dt < cand[0]:
                    cand = (dt, j)
        if cand is None or cand[0] > tol + 1e-9:
            continue
        ib = order[cand[1]]
        if ib not in best or cand[0] < best[ib][0]:
            best[ib] = (cand[0], ia)
    return sorted((ia, ib) for ib, (_, ia) in best.items())


def quantile(values: Sequence[float], q: float) -> Optional[float]:
    """线性插值分位数(与 numpy.percentile 默认一致)。"""
    if not values:
        return None
    v = sorted(values)
    pos = q * (len(v) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (pos - lo)


def stats(values: Sequence[float]) -> dict:
    if not values:
        return {"n": 0, "median": None, "p95": None, "max": None}
    return {"n": len(values), "median": quantile(values, 0.5), "p95": quantile(values, 0.95),
            "max": max(values)}


def mean_std(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    n = len(values)
    if n == 0:
        return (None, None)
    m = sum(values) / n
    if n == 1:
        return (m, None)
    return (m, math.sqrt(sum((x - m) ** 2 for x in values) / (n - 1)))


def linear_fit(x: Sequence[float], y: Sequence[float]) -> Optional[Tuple[float, float]]:
    """最小二乘 y = a + b·x,返回 (a, b);x 无变化时 None。"""
    n = len(x)
    if n < 2:
        return None
    mx = sum(x) / n
    my = sum(y) / n
    sxx = sum((xi - mx) ** 2 for xi in x)
    if sxx <= 1e-12:
        return None
    b = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y)) / sxx
    return (my - b * mx, b)


# ---------------------------------------------------------------------------
# 大地坐标
# ---------------------------------------------------------------------------
def llh_to_ecef(lat_deg: float, lon_deg: float, h: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1.0 - WGS84_E2) + h) * math.sin(lat))


def enu_offset(lat0: float, lon0: float, h0: float, lat: float, lon: float, h: float) -> Tuple[float, float, float]:
    """点 (lat,lon,h) 相对参考点 (lat0,lon0,h0) 的 ENU 偏移(m),在参考点处旋转。"""
    x0, y0, z0 = llh_to_ecef(lat0, lon0, h0)
    x, y, z = llh_to_ecef(lat, lon, h)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = math.radians(lat0), math.radians(lon0)
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    u = math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy + math.sin(la) * dz
    return (e, n, u)


def body_decompose(de: float, dn: float, heading_deg: float) -> Tuple[float, float]:
    """ENU 水平差 → 车体系 (前, 右);航向北零顺时针。"""
    h = math.radians(heading_deg)
    return (de * math.sin(h) + dn * math.cos(h), de * math.cos(h) - dn * math.sin(h))


def course_deg(de: float, dn: float) -> float:
    """位移 (de, dn) 的航迹向,北零顺时针 [0, 360)。"""
    return math.degrees(math.atan2(de, dn)) % 360.0


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def heading_circular_std_deg(headings: Sequence[float]) -> Optional[float]:
    """航向的圆周标准差(°):sqrt(-2 ln R),R 为单位向量平均长度;跨 0° 正确处理。"""
    if not headings:
        return None
    c = sum(math.cos(math.radians(h)) for h in headings) / len(headings)
    s_ = sum(math.sin(math.radians(h)) for h in headings) / len(headings)
    r = min(1.0, math.hypot(c, s_))
    if r <= 1e-12:
        return float("inf")
    return math.degrees(math.sqrt(max(0.0, -2.0 * math.log(r))))


# 判定参数:航向圆周标准差至少这么大,车体系与地理系的偏移才分得开(直线行驶两者无法区分)
MIN_HEADING_CIRC_STD_DEG = 30.0
MIN_OFFSET_SAMPLES = 30
SMALL_OFFSET_M = 0.2


def classify_offset(de: Sequence[float], dn: Sequence[float], headings: Sequence[float]) -> Tuple[str, str]:
    """水平差 (de, dn) 与对应航向 → (类别, 说明)。
    类别:too_few | insufficient_heading | small | body_fixed | geo_fixed | inconclusive。
    body_fixed 要求:航向圆周标准差 ≥ MIN_HEADING_CIRC_STD_DEG、车体系均值合 > SMALL_OFFSET_M、
    车体系标准差合 < 均值合的一半、且小于 ENU 标准差合。"""
    n = len(headings)
    if n < MIN_OFFSET_SAMPLES:
        return "too_few", f"样本 {n} < {MIN_OFFSET_SAMPLES},无法判定"
    fw, rt = [], []
    for e, nn, h in zip(de, dn, headings):
        f_, r_ = body_decompose(e, nn, h)
        fw.append(f_)
        rt.append(r_)
    mf, sf = mean_std(fw)
    mr, sr = mean_std(rt)
    me, se = mean_std(list(de))
    mn, sn = mean_std(list(dn))
    body_std = math.hypot(sf, sr)
    enu_std = math.hypot(se, sn)
    mag = math.hypot(mf, mr)
    cstd = heading_circular_std_deg(headings)
    if mag <= SMALL_OFFSET_M and math.hypot(me, mn) <= SMALL_OFFSET_M:
        return "small", f"车体系均值合 {mag:.3f} m ≤ {SMALL_OFFSET_M} m,未见明显偏移"
    if cstd < MIN_HEADING_CIRC_STD_DEG:
        return "insufficient_heading", (f"航向圆周标准差 {cstd:.1f}° < {MIN_HEADING_CIRC_STD_DEG:.0f}°,航向变化不足,"
                                        "分不清车体系固定(杆臂)与地理系固定偏移")
    if mag > SMALL_OFFSET_M and body_std < 0.5 * mag and body_std < enu_std:
        return "body_fixed", (f"**提示存在车体系固定偏移(杆臂)**:车体系均值 前 {mf:.3f} / 右 {mr:.3f} m(合 {mag:.3f} m),"
                              f"车体系标准差合 {body_std:.3f} m < ENU 标准差合 {enu_std:.3f} m,航向圆周标准差 {cstd:.1f}°")
    if enu_std < body_std and math.hypot(me, mn) > SMALL_OFFSET_M:
        return "geo_fixed", (f"**偏移不随航向转动**:ENU 均值 东 {me:.3f} / 北 {mn:.3f} m,ENU 标准差合 {enu_std:.3f} m "
                             f"< 车体系标准差合 {body_std:.3f} m,更像地理系固定偏移(基准/基站坐标差)而非杆臂")
    return "inconclusive", (f"车体系均值合 {mag:.3f} m、标准差合 {body_std:.3f} m,ENU 标准差合 {enu_std:.3f} m,"
                            "不满足任一判据")


# ---------------------------------------------------------------------------
# events.log
# ---------------------------------------------------------------------------
@dataclass
class EventLine:
    raw: str
    t: float
    kind: str            # OPEN | CLOSE
    level: str
    code: str
    lat: Optional[float]
    lon: Optional[float]
    message: str
    opened_t: Optional[float] = None
    duration_s: Optional[float] = None
    reason: Optional[str] = None
    peak: Dict[str, float] = field(default_factory=dict)


@dataclass
class EventInterval:
    code: str
    level: str
    t_open: float
    t_close: Optional[float]
    reason: Optional[str]
    peak: Dict[str, float]
    message: str


def _opt_float(v: str) -> Optional[float]:
    try:
        return float(v)
    except ValueError:
        return None


def parse_events(lines: Sequence[str]) -> List[EventLine]:
    out: List[EventLine] = []
    for raw in lines:
        s = raw.rstrip("\n")
        if not s.strip() or s.lstrip().startswith("%"):
            continue
        tok = s.split(" ")
        tok = [x for x in tok if x != ""]
        if len(tok) < 7 or tok[2] not in ("OPEN", "CLOSE"):
            continue
        t = _calendar_to_unix(tok[0], tok[1])
        if t is None:
            continue
        ev = EventLine(raw=s, t=t, kind=tok[2], level=tok[3], code=tok[4], lat=None, lon=None, message="")
        i = 5
        while i < len(tok):
            x = tok[i]
            if x.startswith("lat="):
                ev.lat = _opt_float(x[4:])
            elif x.startswith("lon="):
                ev.lon = _opt_float(x[4:])
            elif x.startswith("opened=") and i + 1 < len(tok):
                ev.opened_t = _calendar_to_unix(x[7:], tok[i + 1])
                i += 1
            elif x.startswith("duration_s="):
                ev.duration_s = _opt_float(x[11:])
            elif x.startswith("reason="):
                ev.reason = x[7:]
            elif x.startswith("peak="):
                body = x[5:]
                if body != "-":
                    for kv in body.split(";"):
                        if "=" in kv:
                            k, v = kv.split("=", 1)
                            fv = _opt_float(v)
                            if fv is not None:
                                ev.peak[k] = fv
            else:
                break
            i += 1
        ev.message = " ".join(tok[i:])
        out.append(ev)
    return out


def event_intervals(events: Sequence[EventLine]) -> List[EventInterval]:
    open_by_code: Dict[str, List[EventLine]] = {}
    out: List[EventInterval] = []
    for ev in events:
        if ev.kind == "OPEN":
            open_by_code.setdefault(ev.code, []).append(ev)
            continue
        stack = open_by_code.get(ev.code, [])
        match = None
        for o in stack:
            if ev.opened_t is None or abs(o.t - ev.opened_t) < 0.01:
                match = o
                break
        if match is not None:
            stack.remove(match)
            out.append(EventInterval(ev.code, match.level, match.t, ev.t, ev.reason, ev.peak, match.message))
        else:
            out.append(EventInterval(ev.code, ev.level, ev.opened_t if ev.opened_t is not None else ev.t,
                                     ev.t, ev.reason, ev.peak, ev.message))
    for stack in open_by_code.values():
        for o in stack:
            out.append(EventInterval(o.code, o.level, o.t, None, None, {}, o.message))
    out.sort(key=lambda x: x.t_open)
    return out


# ---------------------------------------------------------------------------
# 录包
# ---------------------------------------------------------------------------
@dataclass
class BagData:
    path: str
    error: Optional[str] = None
    fix_t: List[float] = field(default_factory=list)          # /gnss_cgi610/rtk_fix gnss_time(UTC)
    fix_llh: List[Tuple[float, float, float]] = field(default_factory=list)
    fix_heading: List[Optional[float]] = field(default_factory=list)   # heading_valid=false → None
    gnss_diag: List[Tuple[float, int, str, str]] = field(default_factory=list)   # (stamp, level, status_code, message)
    rtkrcv_diag: List[Tuple[float, int, str]] = field(default_factory=list)      # (stamp, level, message)


def read_bag(path: str) -> BagData:
    bd = BagData(path=path)
    try:
        import rosbag2_py  # noqa
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as e:  # noqa: BLE001
        bd.error = f"无法导入 rosbag2_py/rclpy({e});先 source ROS 与 overlay"
        return bd
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        want = [x for x in ("/gnss_cgi610/rtk_fix", "/gnss/diagnostics", "/rtkrcv_node/diagnostics") if x in types]
        reader.set_filter(rosbag2_py.StorageFilter(topics=want))
        cls = {k: get_message(types[k]) for k in want}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            m = deserialize_message(data, cls[topic])
            if topic == "/gnss_cgi610/rtk_fix":
                bd.fix_t.append(float(m.gnss_time))
                bd.fix_llh.append((m.latitude, m.longitude, m.altitude))
                bd.fix_heading.append(float(m.heading) if m.heading_valid else None)
            else:
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                for st in m.status:
                    lvl = st.level[0] if isinstance(st.level, (bytes, bytearray)) else int(st.level)
                    if topic == "/gnss/diagnostics":
                        code = next((kv.value for kv in st.values if kv.key == "status_code"), "")
                        bd.gnss_diag.append((stamp, lvl, code, st.message))
                    else:
                        bd.rtkrcv_diag.append((stamp, lvl, st.message))
        for k in ("/gnss_cgi610/rtk_fix", "/gnss/diagnostics", "/rtkrcv_node/diagnostics"):
            if k not in types:
                bd.error = (bd.error + ";" if bd.error else "") + f"录包里没有 {k}"
    except Exception as e:  # noqa: BLE001
        bd.error = f"读录包失败:{e}"
    return bd


# ---------------------------------------------------------------------------
# Markdown 辅助
# ---------------------------------------------------------------------------
def fnum(v, nd=3) -> str:
    return "—" if v is None else f"{v:.{nd}f}"


def ftime(t: Optional[float], gpst: bool = False) -> str:
    """UTC unix 秒 → "HH:MM:SS.mmm";先整体四舍五入到毫秒再拆秒,x.9996 进位到下一秒。"""
    if t is None:
        return "—"
    import time as _t
    total_ms = int(round((t + (GPS_UTC_LEAP if gpst else 0)) * 1000.0))
    whole, ms = divmod(total_ms, 1000)
    return _t.strftime("%H:%M:%S", _t.gmtime(whole)) + f".{ms:03d}"


def table(header: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(x) for x in r) + " |")
    return "\n".join(out)


LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def normalize_msg(m: str) -> str:
    """诊断消息里的数字(秒数、颗数、米数)归一成 N,便于按类计数。"""
    import re
    return re.sub(r"\d+(\.\d+)?(?=\s?(s|颗|m|%))", "N", m)


# ---------------------------------------------------------------------------
# 评估
# ---------------------------------------------------------------------------
@dataclass
class Diff:
    t: float
    q_src: int
    q_ref: int
    de: float
    dn: float
    du: float
    ns_src: int
    ns_ref: int

    @property
    def horiz(self) -> float:
        return math.hypot(self.de, self.dn)


def diffs(src: List[PosRecord], ref: List[PosRecord], tol: float) -> List[Diff]:
    out = []
    for ia, ib in pair_by_time([r.t for r in src], [r.t for r in ref], tol):
        s, r = src[ia], ref[ib]
        e, n, u = enu_offset(r.lat, r.lon, r.height, s.lat, s.lon, s.height)
        out.append(Diff(s.t, s.q, r.q, e, n, u, s.ns, r.ns))
    return out


def first_fix(recs: List[PosRecord]) -> Optional[PosRecord]:
    return next((r for r in recs if r.q == 1), None)


def find_one(pattern: str) -> Optional[str]:
    m = sorted(glob.glob(pattern))
    return m[0] if m else None


def diff_stats_rows(label: str, ds: List[Diff]) -> List[str]:
    h = stats([d.horiz for d in ds])
    v = stats([abs(d.du) for d in ds])
    mu = mean_std([d.du for d in ds])[0]
    return [label, str(h["n"]), fnum(h["median"]), fnum(h["p95"]), fnum(h["max"]),
            fnum(v["median"]), fnum(v["p95"]), fnum(v["max"]), fnum(mu)]


DIFF_HEADER = ["口径", "配对数", "水平 中位数 m", "水平 p95 m", "水平 max m",
               "|高程差| 中位数 m", "|高程差| p95 m", "|高程差| max m", "高程差均值 m(源−ref)"]


def nearest_index(ts: List[float], t: float, tol: float) -> Optional[int]:
    k = bisect.bisect_left(ts, t)
    best = None
    for j in (k - 1, k):
        if 0 <= j < len(ts) and abs(ts[j] - t) <= tol and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
            best = j
    return best


class EvalInputError(Exception):
    """输入不可用(例如 ref 没有记录):main() 打印原因并返回 2,不输出半截报告。"""


def evaluate(run: str, ref_path: str, bag_path: Optional[str], out_path: str, label: str, tol: float) -> str:
    L: List[str] = []
    can_path = find_one(os.path.join(run, "pos", "*", "can.pos"))
    rtk_path = find_one(os.path.join(run, "pos", "*", "rtkrcv.pos"))
    ev_path = find_one(os.path.join(run, "diag", "*", "events.log"))
    basepos_path = find_one(os.path.join(run, "diag", "*", "base.pos"))
    baseline_path = os.path.join(run, "diag", "base_baseline")
    conf_path = os.path.join(run, "rtkrcv", "rtkrcv.conf")
    if bag_path is None:
        bags = sorted(glob.glob(os.path.join(run, "bags", "gnss_*")))
        bag_path = bags[0] if len(bags) == 1 else None

    ref = read_pos(ref_path)
    if not ref.records:
        # 后面的配对、首次固定、数据起止时刻都以 ref 为基准;没有记录时直接报清楚,不要在中途抛 IndexError
        raise EvalInputError(f"ref 文件里没有可解析的解算记录: {ref_path}"
                             "(只支持经纬高格式、日期时间列或 GPS 周 + 周内秒列的 .pos)")
    can = read_pos(can_path) if can_path else None
    rtk = read_pos(rtk_path) if rtk_path else None
    srcs = OrderedDict([("can", can), ("rtkrcv", rtk), ("ref", ref)])
    data_t0 = ref.records[0].t if ref.records else None
    data_end = max([p.records[-1].t for p in (can, ref) if p and p.records], default=None)

    L.append(f"# field_eval —— run {label}\n")
    L.append("由 `gnss_bringup/tools/field_replay/field_eval.py` 生成。时间一律 UTC(括号内或标注处为 GPST = UTC+18 s)。"
             f"位置差 = 源 − ref,在 ref 点当地 ENU 下计算;时间配对容差 {tol} s,一对一取最近。\n")
    L.append("## 0. 输入\n")
    rows = []
    for name, p in [("run 目录", run), ("ref", ref_path), ("can.pos", can_path), ("rtkrcv.pos", rtk_path),
                    ("events.log", ev_path), ("base.pos", basepos_path),
                    ("base_baseline", baseline_path if os.path.exists(baseline_path) else None),
                    ("rtkrcv.conf", conf_path if os.path.exists(conf_path) else None), ("录包", bag_path)]:
        rows.append([name, f"`{p}`" if p else "(不存在)"])
    L.append(table(["输入", "路径"], rows) + "\n")
    if os.path.exists(conf_path):
        keys = ("pos1-navsys", "pos1-elmask", "pos2-arelmask", "pos2-armode", "misc-timeout", "misc-reconnect",
                "inpstr1-format", "inpstr2-format")
        got = []
        with open(conf_path, errors="replace") as f:
            for line in f:
                k = line.split("=", 1)[0].strip()
                if k in keys:
                    got.append(line.split("#", 1)[0].strip())
        missing = [k for k in keys if not any(g.startswith(k) for g in got)]
        L.append("rtkrcv.conf 关键行:\n\n```\n" + "\n".join(got) + "\n```\n")
        if missing:
            L.append("conf 中没有这些键(取 RTKLIB 默认):" + "、".join(f"`{k}`" for k in missing) + "\n")
    L.append(f"数据起点(ref 首历元)UTC {ftime(data_t0)};数据终点(can/ref 末历元)UTC {ftime(data_end)}。\n")

    # 1. 各源概况 -----------------------------------------------------------
    L.append("## 1. 各源概况\n")
    rows = []
    for name, p in srcs.items():
        if p is None:
            rows.append([name, "(文件不存在)"] + [""] * 8)
            continue
        rs = p.records
        qc = Counter(r.q for r in rs)
        n = len(rs)
        ff = first_fix(rs)
        rows.append([
            name, str(n),
            f"{ftime(rs[0].t)} ({ftime(rs[0].t, True)})" if rs else "—",
            f"{ftime(rs[-1].t)} ({ftime(rs[-1].t, True)})" if rs else "—",
            fnum(rs[-1].t - rs[0].t, 1) if rs else "—",
            " / ".join(f"Q{q}:{qc[q]}" for q in sorted(qc)) or "—",
            f"{100.0 * qc.get(1, 0) / n:.1f}%" if n else "—",
            fnum(rs[0].t - data_t0, 1) if rs and data_t0 is not None else "—",
            fnum(ff.t - rs[0].t, 1) if ff else "从未固定",
            f"{min(r.ns for r in rs)}–{max(r.ns for r in rs)}" if rs else "—",
        ])
    L.append(table(["源", "历元数", "首历元 UTC (GPST)", "末历元 UTC (GPST)", "覆盖 s", "Q 分布",
                    "固定率", "首解相对数据起点 s", "首次固定相对首解 s", "ns 范围"], rows) + "\n")
    L.append("Q:1 fix、2 float、4 dgps、5 single。can 的 Q 由 610 satellite_status 映射(板卡融合解标签,"
             "不等于天线处本历元 RTK 状态);can 的 σ 列是回放时补的全零 σ 帧,无意义。\n")

    ref_fix = [r for r in ref.records if r.q == 1]

    # 2. rtkrcv vs ref -------------------------------------------------------
    L.append("## 2. rtkrcv 对 ref\n")
    if rtk and rtk.records:
        d_all = diffs(rtk.records, ref.records, tol)
        rows = [diff_stats_rows("双方都 FIXED(主口径)", [d for d in d_all if d.q_src == 1 and d.q_ref == 1])]
        for q in sorted({d.q_src for d in d_all}):
            rows.append(diff_stats_rows(f"rtkrcv Q{q} / ref FIXED", [d for d in d_all if d.q_src == q and d.q_ref == 1]))
        rows.append(diff_stats_rows("全部配对(不限 Q)", d_all))
        L.append(table(DIFF_HEADER, rows) + "\n")
        ff = first_fix(rtk.records)
        L.append(f"- 配对 {len(d_all)} / rtkrcv {len(rtk.records)} 条 / ref {len(ref.records)} 条。")
        L.append(f"- rtkrcv 首解 UTC {ftime(rtk.records[0].t)},首次固定 "
                 + (f"UTC {ftime(ff.t)}(相对首解 +{ff.t - rtk.records[0].t:.1f} s,相对数据起点 +{ff.t - data_t0:.1f} s)"
                    if ff else "无") + "。")
        rff = first_fix(ref.records)
        if rff:
            L.append(f"- 对照:ref 首次固定 UTC {ftime(rff.t)}(相对数据起点 +{rff.t - data_t0:.1f} s)。")
        # Q 一致性
        both = Counter((d.q_src, d.q_ref) for d in d_all)
        L.append("- 配对历元的 (rtkrcv Q, ref Q) 计数:" + "、".join(f"({a},{b}):{n}" for (a, b), n in sorted(both.items())) + "\n")
    else:
        d_all = []
        L.append("rtkrcv.pos 不存在或为空。\n")

    # 3. can vs ref ----------------------------------------------------------
    L.append("## 3. can(610 融合解)对 ref\n")
    bag = read_bag(bag_path) if bag_path else None
    d_can: List[Diff] = []
    can_resid: Dict[float, float] = {}
    lever_body = None
    if can and can.records:
        d_can = diffs(can.records, ref.records, tol)
        rows = []
        for q in sorted({d.q_src for d in d_can}):
            rows.append(diff_stats_rows(f"can Q{q} / ref FIXED", [d for d in d_can if d.q_src == q and d.q_ref == 1]))
        rows.append(diff_stats_rows("can 任意 / ref FIXED", [d for d in d_can if d.q_ref == 1]))
        rows.append(diff_stats_rows("全部配对(不限 Q)", d_can))
        L.append(table(DIFF_HEADER, rows) + "\n")
        dref = [d for d in d_can if d.q_ref == 1]
        me, se = mean_std([d.de for d in dref])
        mn, sn = mean_std([d.dn for d in dref])
        mu, su = mean_std([d.du for d in dref])
        L.append(f"ENU 分量(can − ref,ref FIXED,n={len(dref)}):东 均值 {fnum(me)} / 标准差 {fnum(se)} m;"
                 f"北 {fnum(mn)} / {fnum(sn)} m;天 {fnum(mu)} / {fnum(su)} m。\n")

        # 杆臂分析
        L.append("### 3.1 杆臂分析(车体系分解)\n")
        heading_src = None
        headings: Dict[float, float] = {}
        speeds: Dict[float, float] = {}
        cogs: Dict[float, float] = {}
        if bag and bag.fix_t and not (bag.error and "rtk_fix" in bag.error):
            order = sorted(range(len(bag.fix_t)), key=lambda i: bag.fix_t[i])
            bt = [bag.fix_t[i] for i in order]
            bllh = [bag.fix_llh[i] for i in order]
            bh = [bag.fix_heading[i] for i in order]
            for d in dref:
                j = nearest_index(bt, d.t, 0.03)
                if j is None:
                    continue
                j0 = nearest_index(bt, d.t - 0.5, 0.05)
                j1 = nearest_index(bt, d.t + 0.5, 0.05)
                if j0 is not None and j1 is not None and bt[j1] > bt[j0]:
                    e, n, _ = enu_offset(*bllh[j0], *bllh[j1])
                    speeds[d.t] = math.hypot(e, n) / (bt[j1] - bt[j0])
                    if speeds[d.t] > 1.0:
                        cogs[d.t] = course_deg(e, n)
                if bh[j] is not None:
                    headings[d.t] = bh[j]
            heading_src = (f"录包 `/gnss_cgi610/rtk_fix.heading`(610 航向,heading_valid=true,按 gnss_time 与 can.pos "
                           f"历元配对,容差 0.03 s);速度由同话题 50 Hz 位置在 t±0.5 s 的位移求得")
        if not headings:
            # 退回:can.pos 相邻历元航迹向,仅速度 > 1 m/s
            why = bag.error if bag and bag.error else ("未给录包" if not bag else "录包里无有效航向")
            heading_src = f"can.pos 相邻历元航迹向(速度 > 1 m/s 才用;原因:{why})"
            cr = can.records
            for i in range(1, len(cr) - 1):
                dt = cr[i + 1].t - cr[i - 1].t
                if dt <= 0:
                    continue
                e, n, _ = enu_offset(cr[i - 1].lat, cr[i - 1].lon, cr[i - 1].height, cr[i + 1].lat, cr[i + 1].lon, cr[i + 1].height)
                v = math.hypot(e, n) / dt
                speeds[cr[i].t] = v
                if v > 1.0:
                    headings[cr[i].t] = course_deg(e, n)
                    cogs[cr[i].t] = headings[cr[i].t]
        L.append(f"航向来源:{heading_src}。\n")

        def body_rows(sel_name, pred):
            fw, rt, de_, dn_, sp = [], [], [], [], []
            for d in dref:
                if d.t not in headings or not pred(d):
                    continue
                f_, r_ = body_decompose(d.de, d.dn, headings[d.t])
                fw.append(f_)
                rt.append(r_)
                de_.append(d.de)
                dn_.append(d.dn)
                sp.append(speeds.get(d.t))
            mf, sf = mean_std(fw)
            mr, sr = mean_std(rt)
            _, se_ = mean_std(de_)
            _, sn_ = mean_std(dn_)
            return [sel_name, str(len(fw)), fnum(mf), fnum(sf), fnum(mr), fnum(sr), fnum(se_), fnum(sn_)], (fw, rt, sp, sf, sr, se_, sn_, mf, mr)

        r_all, info_all = body_rows("全部(有航向)", lambda d: True)
        r_mov, info_mov = body_rows("速度 > 1 m/s", lambda d: speeds.get(d.t, 0.0) > 1.0)
        r_sta, info_sta = body_rows("速度 < 0.2 m/s", lambda d: d.t in speeds and speeds[d.t] < 0.2)
        L.append(table(["子集", "n", "前 均值 m", "前 标准差 m", "右 均值 m", "右 标准差 m", "东 标准差 m", "北 标准差 m"],
                       [r_all, r_mov, r_sta]) + "\n")
        # 速度回归:前向差 = a + b·v,b 相当于时间偏差(s)
        fw, _, sp, *_ = info_all
        xy = [(v, f_) for v, f_ in zip(sp, fw) if v is not None]
        fit = linear_fit([x for x, _ in xy], [y for _, y in xy]) if xy else None
        if fit:
            L.append(f"前向差对速度线性回归(n={len(xy)}):前 = {fit[0]:.3f} m + {fit[1]:.4f} s × 速度。"
                     "斜率相当于 610 融合解相对 ref 的时间偏差(正 = 610 超前);截距是零速时的前向偏移。\n")
        if cogs and not heading_src.startswith("can.pos"):   # 退回航迹向时航向就是航迹向,比较无意义
            dh = [wrap180(headings[t] - cogs[t]) for t in cogs if t in headings]
            if dh:
                L.append(f"航向 − 航迹向(速度 > 1 m/s,n={len(dh)}):中位数 {fnum(quantile(dh, 0.5), 2)}°,"
                         f"|差| p95 {fnum(quantile([abs(x) for x in dh], 0.95), 2)}°(大于 90° 的多为倒车)。\n")
        # 判定
        fw, rt, sp, sf, sr, se_, sn_, mf, mr = info_all
        sel = [d for d in dref if d.t in headings]
        kind, verdict = classify_offset([d.de for d in sel], [d.dn for d in sel], [headings[d.t] for d in sel])
        L.append(f"自动判定(判据:航向圆周标准差 ≥ {MIN_HEADING_CIRC_STD_DEG:.0f}°、车体系均值合 > {SMALL_OFFSET_M} m、"
                 "车体系标准差合 < 均值合的一半且小于 ENU 标准差合 → 车体系固定偏移):" + verdict + "。\n")
        if kind == "body_fixed":
            lever_body = (mf, mr)
            for d in dref:
                if d.t in headings:
                    h = math.radians(headings[d.t])
                    pe = mf * math.sin(h) + mr * math.cos(h)
                    pn = mf * math.cos(h) - mr * math.sin(h)
                    can_resid[d.t] = math.hypot(d.de - pe, d.dn - pn)
            rs = stats(list(can_resid.values()))
            L.append(f"扣除上面的车体系固定偏移(前 {mf:.3f} / 右 {mr:.3f} m)后,can − ref 水平残差:n={rs['n']}、"
                     f"中位 {fnum(rs['median'])} m、p95 {fnum(rs['p95'])} m、max {fnum(rs['max'])} m"
                     f"(同一批数据既估偏移又算残差,属自洽检查,不是独立验证)。\n")
    else:
        L.append("can.pos 不存在或为空。\n")

    # 4. 基站坐标 ------------------------------------------------------------
    L.append("## 4. 基站坐标\n")
    if ref.ref_llh:
        rx = llh_to_ecef(*ref.ref_llh)
        L.append(f"ref pos(`{os.path.basename(ref_path)}` 头部):{ref.ref_llh[0]:.9f} {ref.ref_llh[1]:.9f} {ref.ref_llh[2]:.4f}"
                 f" → ECEF ({rx[0]:.4f}, {rx[1]:.4f}, {rx[2]:.4f})。\n")
        rows = []
        if basepos_path:
            with open(basepos_path) as f:
                for line in f:
                    c = line.split()
                    if not c or c[0].startswith("%") or len(c) < 5:
                        continue
                    try:
                        x, y, z = float(c[2]), float(c[3]), float(c[4])
                    except ValueError:
                        continue
                    rows.append([f"base.pos {c[0]} {c[1]} UTC", f"({x:.4f}, {y:.4f}, {z:.4f})",
                                 f"{math.dist((x, y, z), rx):.4f}"])
        if os.path.exists(baseline_path):
            with open(baseline_path) as f:
                s = f.readline().strip()
            try:
                x, y, z = (float(v) for v in s.split(","))
                rows.append(["diag/base_baseline", f"({x:.4f}, {y:.4f}, {z:.4f})", f"{math.dist((x, y, z), rx):.4f}"])
            except ValueError:
                rows.append(["diag/base_baseline", f"无法解析:`{s}`", "—"])
        else:
            rows.append(["diag/base_baseline", "(文件不存在)", "—"])
        L.append(table(["来源", "ECEF m", "与 ref pos 距离 m"], rows) + "\n")
    else:
        L.append("ref 文件头部没有 ref pos,跳过。\n")

    # 5. 事件 ----------------------------------------------------------------
    L.append("## 5. 诊断事件(events.log)\n")
    if ev_path:
        with open(ev_path, errors="replace") as f:
            ev_lines = f.readlines()
        evs = parse_events(ev_lines)
        ivs = event_intervals(evs)
        rows = []
        by_code: "OrderedDict[str, List[EventInterval]]" = OrderedDict()
        for iv in sorted(ivs, key=lambda x: x.code):
            by_code.setdefault(iv.code, []).append(iv)
        for code, lst in by_code.items():
            durs = [(iv.t_close if iv.t_close is not None else data_end) - iv.t_open for iv in lst]
            reasons = Counter(iv.reason or "未关闭" for iv in lst)
            after = sum(1 for iv in lst if data_end is not None and iv.t_open > data_end)
            rows.append([code, "/".join(sorted({iv.level for iv in lst})), str(len(lst)), fnum(sum(durs), 1),
                         fnum(max(durs), 1), "、".join(f"{k}×{v}" for k, v in reasons.items()), str(after)])
        L.append(table(["代码", "级别", "次数", "总时长 s", "最长 s", "关闭原因", "数据结束后才打开"], rows) + "\n")
        L.append(f"(OPEN {sum(1 for e in evs if e.kind == 'OPEN')} 行,CLOSE {sum(1 for e in evs if e.kind == 'CLOSE')} 行。"
                 "reason=shutdown 是节点退出时强制关闭,时长截到退出时刻。)\n")

        L.append("### 5.1 逐事件核对\n")
        L.append("依据:同期(事件打开到关闭,且截到数据终点)ref 与 rtkrcv/can 的实际情况。"
                 "判断只说明'事件描述的现象/原因与数据是否相符',不是对诊断算法的整体结论。\n")
        rows = []
        for iv in ivs:
            t1 = iv.t_close if iv.t_close is not None else data_end
            t1c = min(t1, data_end) if data_end is not None else t1
            in_iv = lambda t: iv.t_open <= t <= t1c  # noqa: E731
            ref_in = [r for r in ref.records if in_iv(r.t)]
            ref_fix_rate = (100.0 * sum(1 for r in ref_in if r.q == 1) / len(ref_in)) if ref_in else None
            rtk_in = [r for r in (rtk.records if rtk else []) if in_iv(r.t)]
            can_d = [d.horiz for d in d_can if in_iv(d.t) and d.q_ref == 1]
            can_r = [v for t, v in can_resid.items() if in_iv(t)]
            rtk_d = [d.horiz for d in d_all if in_iv(d.t) and d.q_ref == 1]
            ctx = []
            if ref_in:
                ctx.append(f"ref 固定率 {ref_fix_rate:.0f}%、ns 中位 {quantile([r.ns for r in ref_in], 0.5):.0f}")
            if rtk_in:
                qc = Counter(r.q for r in rtk_in)
                ctx.append(f"rtkrcv {len(rtk_in)} 条(" + "/".join(f"Q{q}:{qc[q]}" for q in sorted(qc))
                           + f")、ns 中位 {quantile([r.ns for r in rtk_in], 0.5):.0f}")
            else:
                ctx.append("rtkrcv 0 条")
            if can_d:
                ctx.append(f"can−ref 水平中位 {quantile(can_d, 0.5):.2f} m")
            if rtk_d:
                ctx.append(f"rtkrcv−ref 水平中位 {quantile(rtk_d, 0.5):.2f} m")
            rtk_before = [r for r in (rtk.records if rtk else []) if iv.t_open - 5.0 <= r.t <= iv.t_open]
            rtk_after_open = [r for r in (rtk.records if rtk else []) if r.t >= iv.t_open]
            verdict, why = judge_event(iv, data_end, ref_in, ref_fix_rate, rtk_in, can_d, rtk_d, can_r,
                                       rtk_before, rtk_after_open[0].t if rtk_after_open else None)
            rows.append([f"{ftime(iv.t_open)}–{ftime(iv.t_close)}", iv.code, iv.level,
                         fnum((iv.t_close - iv.t_open) if iv.t_close is not None else None, 1),
                         iv.reason or "未关闭", ";".join(ctx), f"**{verdict}**:{why}"])
        L.append(table(["UTC 打开–关闭", "代码", "级别", "时长 s", "关闭原因", "同期数据", "判断"], rows) + "\n")
        L.append("### 5.2 events.log 原文\n")
        L.append("```\n" + "".join(ev_lines).rstrip("\n") + "\n```\n")
    else:
        L.append("events.log 不存在。\n")

    # 6. 诊断话题 ------------------------------------------------------------
    L.append("## 6. 诊断话题(录包)\n")
    if bag is None:
        L.append("未找到录包,跳过。\n")
    else:
        if bag.error:
            L.append(f"录包提示:{bag.error}\n")
        if bag.gnss_diag:
            lv = Counter(LEVEL_NAMES.get(x[1], str(x[1])) for x in bag.gnss_diag)
            L.append(f"`/gnss/diagnostics` {len(bag.gnss_diag)} 条状态,级别:"
                     + "、".join(f"{k} {v}" for k, v in sorted(lv.items())) + "。\n")
            cc = Counter((x[2] or "(无 status_code)", LEVEL_NAMES.get(x[1], str(x[1]))) for x in bag.gnss_diag)
            late = Counter((x[2] or "(无 status_code)", LEVEL_NAMES.get(x[1], str(x[1]))) for x in bag.gnss_diag
                           if data_end is not None and x[0] > data_end)
            L.append(table(["status_code", "级别", "条数", "其中数据结束后"],
                           [[c, l, str(n), str(late.get((c, l), 0))] for (c, l), n in sorted(cc.items(), key=lambda kv: -kv[1])]) + "\n")
            L.append("(`/gnss/diagnostics` 的 header.stamp 是 sim time = 数据时间,可以与数据终点比较。)\n")
            nocode = Counter(normalize_msg(x[3]) for x in bag.gnss_diag if not x[2])
            if nocode:
                L.append("无 status_code 的消息文本(数字归一为 N):" + "、".join(f"`{m}`×{n}" for m, n in nocode.most_common(5)) + "\n")
        if bag.rtkrcv_diag:
            lv = Counter(LEVEL_NAMES.get(x[1], str(x[1])) for x in bag.rtkrcv_diag)
            L.append(f"`/rtkrcv_node/diagnostics` {len(bag.rtkrcv_diag)} 条状态,级别:"
                     + "、".join(f"{k} {v}" for k, v in sorted(lv.items())) + "。\n")
            mc = Counter((LEVEL_NAMES.get(x[1], str(x[1])), normalize_msg(x[2])) for x in bag.rtkrcv_diag)
            L.append(table(["级别", "消息(数字归一为 N)", "条数"], [[l, m, str(n)] for (l, m), n in mc.most_common(10)]) + "\n")
            L.append("(`/rtkrcv_node/diagnostics` 的 header.stamp 是墙钟,不与数据时间比较。)\n")
        if bag.fix_t:
            L.append(f"`/gnss_cgi610/rtk_fix` {len(bag.fix_t)} 条,heading_valid=true {sum(1 for h in bag.fix_heading if h is not None)} 条。\n")

    text = "\n".join(L)
    with open(out_path, "w") as f:
        f.write(text)
    return text


def judge_event(iv: EventInterval, data_end: Optional[float], ref_in: List[PosRecord], ref_fix_rate: Optional[float],
                rtk_in: List[PosRecord], can_d: List[float], rtk_d: List[float], can_resid: Sequence[float] = (),
                rtk_before_open: Sequence[PosRecord] = (), first_rtk_after_open: Optional[float] = None) -> Tuple[str, str]:
    """一句话判断事件与同期数据是否相符。返回 (合理|可疑|无法判定, 依据)。"""
    if data_end is not None and iv.t_open > data_end:
        return "可疑", "打开时数据已结束(回放尾部 /clock 空转),不是数据里的现象"
    code = iv.code
    rtk_ns = quantile([r.ns for r in rtk_in], 0.5) if rtk_in else None
    ref_ns = quantile([r.ns for r in ref_in], 0.5) if ref_in else None
    if code == "device_divergence":
        if not can_d:
            return "无法判定", "同期无 can/ref(FIXED)配对"
        cm = quantile(can_d, 0.5)
        rm = quantile(rtk_d, 0.5) if rtk_d else None
        if cm > 1.0 and can_resid and quantile(list(can_resid), 0.5) < 0.5:
            return "现象属实、原因可疑", (f"610 与 ref 实测偏差中位 {cm:.2f} m 属实,但扣除车体系固定偏移(见 3.1)后残差中位 "
                                         f"{quantile(list(can_resid), 0.5):.2f} m——偏差是固定杆臂/输出点,不是'610 融合问题'")
        if cm > 1.0:
            extra = f";rtkrcv−ref 中位 {rm:.2f} m" if rm is not None else ""
            return "合理", f"610 与 ref 实测偏差中位 {cm:.2f} m(> 1 m),偏差确实在 610 一侧{extra}"
        if rm is not None and rm > 1.0:
            return "可疑", f"610 与 ref 只差 {cm:.2f} m,偏差主要来自 rtkrcv 解(与 ref 差 {rm:.2f} m),不是'610 融合问题'"
        return "可疑", f"610 与 ref 只差 {cm:.2f} m、rtkrcv 与 ref 差 {fnum(rm, 2)} m,都不大"
    if code == "low_sats":
        if ref_ns is not None and rtk_ns is not None and ref_ns >= 10 and rtk_ns < 6:
            return "现象属实、原因可疑", (f"rtkrcv ns 中位 {rtk_ns:.0f} 确实少,但同期 ref ns 中位 {ref_ns:.0f},"
                                         "天空不遮挡——卫星少来自星历/输入缺失,不是'疑似遮挡'")
        return ("合理", f"rtkrcv ns 中位 {fnum(rtk_ns, 0)}、ref ns 中位 {fnum(ref_ns, 0)}") if rtk_ns is not None else \
            ("无法判定", "同期无 rtkrcv 解")
    if code == "no_solution":
        if not rtk_in:
            return "合理", "同期 rtkrcv 确实没有解"
        if not rtk_before_open and first_rtk_after_open is not None:
            lag = (iv.t_close - first_rtk_after_open) if iv.t_close is not None else None
            return "合理", (f"打开前 5 s 内确无 rtkrcv 解;rtkrcv 首条解 {ftime(first_rtk_after_open)} 出现后"
                           + (f" {lag:.1f} s 才关闭(关闭滞后)" if lag is not None else "仍未关闭"))
        return "可疑", f"打开前 5 s 内 rtkrcv 有 {len(rtk_before_open)} 条解,同期共 {len(rtk_in)} 条"
    if code == "ambiguity":
        if not rtk_in:
            return "无法判定", "同期无 rtkrcv 解"
        fixed = sum(1 for r in rtk_in if r.q == 1) / len(rtk_in)
        if fixed < 0.5 and ref_fix_rate is not None and ref_fix_rate >= 90:
            return "现象属实、原因可疑", (f"rtkrcv 固定率 {100 * fixed:.0f}% 确实未固定,但同期 ref 固定率 {ref_fix_rate:.0f}%,"
                                         "不是'遮挡过渡区'——是 rtkrcv 自身配置/输入问题")
        if fixed < 0.5:
            return "合理", f"rtkrcv 固定率 {100 * fixed:.0f}%,ref 同期固定率 {fnum(ref_fix_rate, 0)}%"
        return "可疑", f"同期 rtkrcv 固定率 {100 * fixed:.0f}%"
    if code in ("multipath", "cycle_slip"):
        dur = (min(iv.t_close, data_end) if iv.t_close is not None and data_end is not None else (iv.t_close or data_end or iv.t_open)) - iv.t_open
        if rtk_in and rtk_d:
            fixed = sum(1 for r in rtk_in if r.q == 1) / len(rtk_in)
            rm = quantile(rtk_d, 0.5)
            if dur >= 60 and fixed >= 0.95 and rm < 0.05:
                full = (iv.t_close - iv.t_open) if iv.t_close is not None else None
                return "无法判定(解算未受影响)", (f"截至数据终点持续 {dur:.0f} s(事件记录时长 {fnum(full, 1)} s,含数据结束后的尾部),同期 rtkrcv 固定率 {100 * fixed:.0f}%、与 ref 水平中位 {rm:.3f} m;"
                                                 "pos 层面否定不了残差/失锁本身,但'动态遮挡/天线馈线问题'之类原因在解上没有体现")
        return "无法判定", ("pos 层面无法直接验证残差/失锁;同期 ref 固定率 "
                           f"{fnum(ref_fix_rate, 0)}%" + (f"、rtkrcv−ref 水平中位 {quantile(rtk_d, 0.5):.2f} m" if rtk_d else ""))
    if code == "corr_outage":
        return "无法判定", "需对照差分流时间;数据期内打开的请查 base.rtcm3 帧间隔"
    return "无法判定", "未实现该代码的核对规则"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--bag", default=None)
    ap.add_argument("--label", default=None)
    ap.add_argument("--pair-tol", type=float, default=0.1)
    a = ap.parse_args(argv)
    label = a.label or os.path.basename(os.path.normpath(a.run))
    try:
        evaluate(a.run, a.ref, a.bag, a.out, label, a.pair_tol)
    except EvalInputError as e:
        print(f"field_eval.py: {e}", file=sys.stderr)
        return 2
    print(f"written {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
