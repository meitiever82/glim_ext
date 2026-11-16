"""Pure report statistics over stored epochs/events (spec §6)."""
from __future__ import annotations

import bisect
import math

from gnss_hmi.geo import horiz_dist_m
from gnss_hmi.storage.epochs import EpochStore
from gnss_hmi.storage.events import EventStore

# max |Δt| for pairing a can epoch with an rtkrcv epoch in the 610-vs-independent
# deviation stat: wide enough to pair 1 Hz sources with offset phases, tight
# enough that vehicle travel in the gap doesn't dominate the measured deviation.
_DEV_PAIR_TOL_S = 0.5


def _fix_ratio(rows, fixed_q: int) -> float | None:
    if not rows:
        return None
    return sum(1 for e in rows if e.q == fixed_q) / len(rows)


def compute_report(epochs: EpochStore, events: EventStore, t0: float, t1: float,
                   control_points=(), abs_ref_radius_m: float = 3.0) -> dict:
    rtk = epochs.query("rtkrcv", t0, t1)
    can = epochs.query("can", t0, t1)
    gpchc = epochs.query("gpchc", t0, t1)
    main_rows, fixed_q = (rtk, 1) if rtk else (can, 4)
    hourly = []
    if main_rows:
        h0, h1 = int(t0 // 3600), int(t1 // 3600)
        for h in range(h0, h1 + 1):
            rows = [e for e in main_rows if h * 3600 <= e.t < (h + 1) * 3600]
            hourly.append({"hour": h, "epochs": len(rows),
                           "fix_ratio": _fix_ratio(rows, fixed_q)})
    evs = []
    for r in events.query(since=0.0):
        if r.etype != "diagnosis":
            # The events table also stores link/crash rows (etype
            # corr_link/web/rtkrcv/...); the report only summarizes
            # diagnosis outcomes.
            continue
        still_open = r.t_close is None
        if r.t > t1 or ((not still_open) and r.t_close < t0):
            continue
        evs.append({"code": r.code, "level": r.level, "t": r.t, "t_close": r.t_close,
                    "duration_s": (r.t_close - r.t) if r.t_close is not None else None,
                    "message": r.detail, "lat": r.lat, "lon": r.lon})
    hist = [h for h in epochs.base_history() if t0 <= h[0] <= t1]
    base_max = None
    base_series = []
    if hist:
        x0, y0, z0 = hist[0][1:]
        base_series = [{"t": h[0], "offset_m": math.dist((x0, y0, z0), h[1:])} for h in hist]
        base_max = max(s["offset_m"] for s in base_series)

    # 610-fused (can) vs independent (rtkrcv) horizontal deviation (spec §6).
    # Each rtkrcv epoch is paired with the nearest can epoch within
    # _DEV_PAIR_TOL_S; a coarse whole-second bucketing would pair epochs up to
    # ~1 s apart, so a moving vehicle's travel in the gap would dominate the
    # "deviation" instead of the actual 610-vs-independent disagreement.
    can_pos = [e for e in can if e.lat is not None]
    can_pos.sort(key=lambda e: e.t)
    can_ts = [e.t for e in can_pos]
    devs = []
    for e in rtk:
        if e.lat is None or not can_ts:
            continue
        i = bisect.bisect_left(can_ts, e.t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(can_pos)]
        j = min(cand, key=lambda j: abs(can_ts[j] - e.t))
        if abs(can_ts[j] - e.t) <= _DEV_PAIR_TOL_S:
            devs.append(horiz_dist_m(e.lat, e.lon, can_pos[j].lat, can_pos[j].lon))
    can_rtk_dev = {"n": len(devs),
                   "max_m": max(devs) if devs else None,
                   "mean_m": (sum(devs) / len(devs)) if devs else None}

    # Absolute-baseline verification (§6 必含项): for each FIXED rtkrcv epoch
    # (q==1) that was at (within abs_ref_radius_m of) a surveyed control point,
    # the deviation from that point's known coordinate — the last-resort curve
    # that exposes a whole-mine shift the other stats stay green through. Only
    # fixed epochs count: the sub-metre threshold is meaningless against a
    # float/single solution's decimetre-level noise.
    abs_ref = []
    for e in rtk:
        if e.lat is None or e.q != 1:
            continue
        for cp in control_points:
            dev = horiz_dist_m(e.lat, e.lon, cp.lat, cp.lon)
            if dev <= abs_ref_radius_m:
                abs_ref.append({"t": e.t, "cp": cp.name, "dev_m": dev})
    abs_ref_max = max((r["dev_m"] for r in abs_ref), default=None)

    return {"fix_ratio": _fix_ratio(main_rows, fixed_q), "hourly": hourly,
            "events": sorted(evs, key=lambda e: e["t"]),
            "base_max_offset_m": base_max, "base_series": base_series,
            "abs_ref": abs_ref, "abs_ref_max_m": abs_ref_max,
            "can_rtk_dev": can_rtk_dev,
            "epoch_counts": {"rtkrcv": len(rtk), "can": len(can), "gpchc": len(gpchc)}}
