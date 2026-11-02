"""Pure diagnosis rule chain (spec §4.2): first matching rule wins."""
from __future__ import annotations

from dataclasses import dataclass, field

from gnss_hmi.config import DiagnosisCfg
from gnss_hmi.geo import horiz_dist_m
from gnss_hmi.parsers.rtksol import RtkSolution
from gnss_hmi.parsers.rtkstat import SatStat


@dataclass(frozen=True)
class DiagnosisInput:
    now: float
    corr_last_t: float | None          # host time of last RTCM message
    corr_age: float | None             # differential age from solution (s)
    base_offset_m: float | None        # 1005 offset vs learned baseline
    sol: RtkSolution | None            # latest independent solution
    sol_t: float | None                # host time it arrived
    sats: list[SatStat] = field(default_factory=list)
    slip_count_30s: int = 0
    divergence_m: float | None = None      # |610 fused - rtkrcv| horizontal
    divergence_since: float | None = None  # host time divergence first exceeded
    solver_enabled: bool = True
    control_points: list[tuple[float, float]] = field(default_factory=list)  # (lat, lon)


@dataclass(frozen=True)
class Verdict:
    level: str    # ok | info | warning | serious | critical
    code: str
    message: str


def diagnose(inp: DiagnosisInput, cfg: DiagnosisCfg) -> Verdict:
    if inp.sol is None and inp.corr_last_t is None:
        return Verdict("warning", "no_data", "无数据——检查采集链路与设备连接")

    # Rule 1: correction outage / age overrun
    gap = inp.now - inp.corr_last_t if inp.corr_last_t is not None else None
    if (gap is not None and gap > cfg.corr_gap_s) or \
       (inp.corr_age is not None and inp.corr_age > cfg.age_max_s):
        n = int(gap if gap is not None and gap > cfg.corr_gap_s else inp.corr_age)
        return Verdict("serious", "corr_outage",
                       f"差分中断 {n}s——5G 链路或平台转发问题")

    # Rule 2: base station coordinate shift
    if inp.base_offset_m is not None and inp.base_offset_m > cfg.base_shift_m:
        return Verdict("critical", "base_shift",
                       f"⚠ 基站坐标变动 {inp.base_offset_m:.2f}m——全矿定位将整体平移")

    # Rule 2b: absolute-baseline verification against a surveyed control point.
    # When the vehicle is at (within abs_ref_radius_m of) a known control point
    # but the solution deviates past abs_ref_max_m, the whole-mine frame has
    # shifted -- the last-resort check that catches a base-station move the other
    # rules stay green through (spec §4.2). Only the nearest control point within
    # range is judged (the vehicle is "at" one point at a time). Gated to FIXED
    # (q==1) solutions: the 0.2 m threshold is only meaningful against a cm-level
    # fix -- a float/single solution is naturally dm-level and would false-alarm.
    if (inp.sol is not None and inp.sol.q == 1 and inp.sol.lat is not None
            and inp.control_points):
        devs = [(horiz_dist_m(inp.sol.lat, inp.sol.lon, cp[0], cp[1]), cp)
                for cp in inp.control_points]
        near = min(devs, key=lambda d: d[0])
        if near[0] <= cfg.abs_ref_radius_m and near[0] > cfg.abs_ref_max_m:
            return Verdict("critical", "abs_ref_shift",
                           f"⚠ 绝对基准偏差 {near[0]:.2f}m@控制点——全矿整体平移嫌疑")

    # Rule 3: too few satellites
    if inp.sol is not None and inp.sol.ns < cfg.min_sats:
        return Verdict("serious", "low_sats",
                       f"卫星数不足（{inp.sol.ns} 颗）——高帮/坑底遮挡")

    # Rule 4: multipath suspects
    bad = [s for s in inp.sats
           if s.resp > cfg.resid_max_m and (s.el < cfg.low_el_deg or s.snr < cfg.low_snr_dbhz)]
    if len(bad) >= 2:
        names = "、".join(s.sat for s in bad[:4])
        return Verdict("warning", "multipath", f"{names} 残差异常——疑似多路径")

    # Rule 5: ambiguity not fixed
    if inp.sol is not None and inp.sol.q == 2 and inp.sol.ratio < cfg.min_ratio:
        return Verdict("warning", "ambiguity",
                       f"模糊度无法固定（ratio={inp.sol.ratio:.1f}）——遮挡过渡区常见")

    # Rule 6: frequent cycle slips
    if inp.slip_count_30s > cfg.slip_max_per_30s:
        return Verdict("warning", "cycle_slip",
                       "载波频繁失锁——动态遮挡或天线/馈线问题")

    # Rule 7: 610 output diverges from independent solution
    if (inp.divergence_m is not None and inp.divergence_since is not None
            and inp.sol is not None
            and inp.divergence_m > cfg.divergence_sigma * max(
                1e-3, (inp.sol.sdn ** 2 + inp.sol.sde ** 2) ** 0.5)
            and inp.now - inp.divergence_since >= cfg.divergence_hold_s):
        return Verdict("serious", "device_divergence",
                       f"610 输出与独立解算偏差 {inp.divergence_m:.2f}m——疑似 610 融合问题")

    # Check for missing solver output
    if inp.sol is None:
        if inp.solver_enabled:
            return Verdict("warning", "no_solution",
                           "独立解算无输出——rtkrcv 未运行或未收敛")
        return Verdict("info", "no_solution", "独立解算未启用")

    if inp.sol.q != 1:
        return Verdict("info", "not_fixed", f"非固定解（Q={inp.sol.q}）")
    return Verdict("ok", "rtk_fixed", "RTK 固定")
