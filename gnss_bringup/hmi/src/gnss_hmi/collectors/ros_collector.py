"""ROS2 topic collector (read-only) for the GNSS HMI.

Subscribes the single source of truth published by ``gnss_bringup`` and keeps a
thread-safe snapshot for the 1 Hz status loop. Nothing here publishes, calls
services or starts processes — the HMI never writes back to ROS.

Supported kinds:
  rtkfix      gnss_msgs/RtkFix             -> sources[app|fused]
  stat        gnss_msgs/RawStream ($SAT)   -> skyplot satellites
  diagnostic  diagnostic_msgs/DiagnosticArray -> events/banner
  rate        gnss_msgs/RawStream          -> bytes/s + outage for link cards
  gga/navsat/bestpos                       -> legacy Epoch sources (kept)
"""
from __future__ import annotations

import asyncio
import math
import threading
from dataclasses import dataclass
from typing import Callable

from gnss_hmi.parsers.rtkstat import StatEpochAccumulator
from gnss_hmi.storage.epochs import Epoch

OnEpoch = Callable[[Epoch], None]
OnEvent = Callable[[str, str, str], None]

_RATE_WINDOW_S = 10.0
_MAX_LINE_BUF = 256 * 1024
_FINITE_MAX = 1e12


@dataclass(frozen=True)
class TopicCfg:
    topic: str
    kind: str                 # rtkfix | stat | diagnostic | rate | gga | navsat | bestpos
    qos: str = "auto"         # auto | reliable | best_effort
    source: str = ""          # rtkfix: app|fused ; diagnostic: gnss|rtkrcv
    name: str = ""            # rate: corr|obs


def _finite(v) -> float | None:
    """Reject NaN/Inf/absurd values so a bad board value never poisons the UI."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(f) or abs(f) > _FINITE_MAX:
        return None
    return f


def _stamp_s(stamp) -> float | None:
    if stamp is None:
        return None
    sec = int(getattr(stamp, "sec", 0))
    nsec = int(getattr(stamp, "nanosec", 0))
    if sec == 0 and nsec == 0:
        return None
    return sec + nsec * 1e-9


def rtkfix_state(msg, t: float) -> dict | None:
    """gnss_msgs/RtkFix -> UI state dict. None when the position is unusable."""
    lat = _finite(msg.latitude)
    lon = _finite(msg.longitude)
    if lat is None or lon is None:
        return None
    sigma = list(getattr(msg, "sigma_enu", []) or [])
    se = _finite(sigma[0]) if len(sigma) > 0 else None
    sn = _finite(sigma[1]) if len(sigma) > 1 else None
    su = _finite(sigma[2]) if len(sigma) > 2 else None
    ratio = _finite(msg.ratio)
    gnss_time = _finite(msg.gnss_time)
    age = _finite(msg.diff_age)
    heading = _finite(msg.heading)
    heading_valid = bool(getattr(msg, "heading_valid", False)) and heading is not None
    latency = None
    stamp = _stamp_s(getattr(msg, "header", None) and msg.header.stamp)
    if gnss_time is not None and gnss_time > 0 and stamp is not None:
        latency = stamp - gnss_time
    return {
        "t": t,
        "stamp": stamp,
        "q": int(getattr(msg, "quality", 0)),
        "raw_status": int(getattr(msg, "raw_status", 0)),
        "lat": lat, "lon": lon, "alt": _finite(msg.altitude),
        # E/N/U order as published by the board (never rename to sdn/sde)
        "sigma_e": se, "sigma_n": sn, "sigma_u": su,
        "age": age if age is not None and age > 0 else None,
        "sats": int(getattr(msg, "sats_used", 0)) or None,
        "sats_main": int(getattr(msg, "sats_main", 0)) or None,
        "sats_aux": int(getattr(msg, "sats_aux", 0)) or None,
        "heading": heading if heading_valid else None,
        "heading_valid": heading_valid,
        # 0 means "source does not provide" -> None, never a real zero
        "ratio": ratio if ratio is not None and ratio > 0 else None,
        "gnss_time": gnss_time if gnss_time is not None and gnss_time > 0 else None,
        "latency": latency,
        "present": True,
    }


def diag_state(msg, t: float) -> dict:
    """diagnostic_msgs/DiagnosticArray -> {level, message, open_events, verdict...}."""
    statuses = []
    worst = 0
    for st in getattr(msg, "status", []) or []:
        values = {kv.key: kv.value for kv in (getattr(st, "values", []) or [])}
        lvl = int(getattr(st, "level", 0))
        worst = max(worst, lvl)
        statuses.append({
            "name": st.name,
            "level": lvl,
            "level_name": _diag_level_name(lvl),
            "message": st.message,
            "values": values,
        })
    head = statuses[0] if statuses else {}
    values = head.get("values", {})
    open_raw = str(values.get("open_events", "-"))
    open_events = [] if open_raw in ("", "-") else [c for c in open_raw.split(",") if c]
    verdict = {}
    for k, v in values.items():
        if not k.startswith("verdict."):
            continue
        code = k.split(".", 1)[1]
        parts = str(v).split(" ", 1)
        verdict[code] = {"level": parts[0] if parts else "ok",
                         "message": parts[1] if len(parts) > 1 else ""}
    div = values.get("divergence_m")
    try:
        divergence_m = None if div in (None, "-") else float(div)
    except ValueError:
        divergence_m = None
    return {
        "t": t,
        "level": worst,
        "level_name": _diag_level_name(worst),
        "message": head.get("message", ""),
        "status_code": values.get("status_code", ""),
        "divergence_m": divergence_m,
        "open_events": open_events,
        "verdict": verdict,
        "statuses": statuses,
        "present": True,
    }


def _diag_level_name(level: int) -> str:
    # diagnostic_msgs: 0 OK, 1 WARN, 2 ERROR, 3 STALE
    return {0: "ok", 1: "warning", 2: "error", 3: "stale"}.get(level, "ok")


class _LineBuffer:
    """Reassemble ASCII lines from arbitrary RawStream chunks."""

    def __init__(self) -> None:
        self._buf = b""

    def feed(self, data: bytes) -> list[str]:
        if not data:
            return []
        self._buf += bytes(data)
        if len(self._buf) > _MAX_LINE_BUF:      # a runaway stream must not grow forever
            self._buf = self._buf[-_MAX_LINE_BUF:]
        parts = self._buf.split(b"\n")
        self._buf = parts.pop()                # keep the incomplete tail
        return [p.decode("ascii", errors="ignore").strip() for p in parts if p]


class RosCollector:
    """Subscribe ROS topics in a background thread; expose a locked snapshot."""

    def __init__(self, name: str, topics: tuple[TopicCfg, ...],
                 on_epoch: OnEpoch, on_event: OnEvent,
                 timeout_warn_s: float = 5.0) -> None:
        self._name = name
        self._topics = topics
        self._on_epoch = on_epoch
        self._on_event = on_event
        self._timeout_warn_s = timeout_warn_s
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._last_state: str | None = None
        self._lock = threading.RLock()
        self._snapshot = self._empty_snapshot()
        self._line_buf = _LineBuffer()
        self._stat_acc = StatEpochAccumulator()
        self._rate_hits: dict[str, list[tuple[float, int]]] = {}
        self._node = None

    @property
    def name(self) -> str:
        return self._name

    def current_time(self) -> float | None:
        """Node clock (sim time under use_sim_time) for staleness checks."""
        node = self._node
        if node is not None:
            try:
                return node.get_clock().now().nanoseconds * 1e-9
            except Exception:
                pass
        return None

    def snapshot(self) -> dict:
        with self._lock:
            import copy
            return copy.deepcopy(self._snapshot)

    def _empty_snapshot(self) -> dict:
        return {
            "clock_t": None,
            "sources": {"app": None, "fused": None},
            "stat": {"sats": [], "tow": None, "t": None},
            "diagnostics": {"gnss": None, "rtkrcv": None},
            "rates": {"corr": None, "obs": None},
            "topics": {},     # topic -> last message wall time (for "no data" hints)
        }

    async def run(self) -> None:
        self._loop = asyncio.get_running_loop()
        try:
            import rclpy  # noqa: F401
        except ImportError:
            self._event("disabled", "rclpy not available (source ROS first)")
            await self._idle()
            return
        self._thread = threading.Thread(target=self._spin, daemon=True,
                                        name="ros_collector")
        self._thread.start()
        await self._idle()

    async def _idle(self) -> None:
        while True:
            await asyncio.sleep(3600)

    def _event(self, state: str, detail: str) -> None:
        if state != self._last_state:
            self._on_event(self._name, state, detail)
            self._last_state = state

    def _now(self, node) -> float:
        try:
            return node.get_clock().now().nanoseconds * 1e-9
        except Exception:
            import time
            return time.time()

    # --- rclpy thread ------------------------------------------------------
    def _spin(self) -> None:
        import rclpy
        from rclpy.node import Node

        rclpy.init(args=None)
        node = Node("gnss_hmi")
        made = 0
        for tc in self._topics:
            try:
                self._subscribe(node, tc)
                made += 1
            except Exception as e:
                self._event("degraded", f"{tc.topic}: {e}")
        if made == 0:
            self._event("disabled", "no subscriptions created")
        else:
            self._event("connected", f"{made} topic(s)")
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def _qos(self, tc: TopicCfg):
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy)
        if tc.qos == "reliable":
            return QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE)
        if tc.qos == "best_effort":
            return QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        return 10      # ROS default: reliable, depth 10

    def _subscribe(self, node, tc: TopicCfg) -> None:
        qos = self._qos(tc)
        if tc.kind == "rtkfix":
            from gnss_msgs.msg import RtkFix
            src = tc.source or "app"
            cb = lambda msg, s=src, tp=tc.topic: self._on_rtkfix(s, msg, node, tp)
            node.create_subscription(RtkFix, tc.topic, cb, qos)
        elif tc.kind == "stat":
            from gnss_msgs.msg import RawStream
            cb = lambda msg, tp=tc.topic: self._on_stat(msg, node, tp)
            node.create_subscription(RawStream, tc.topic, cb, qos)
        elif tc.kind == "diagnostic":
            from diagnostic_msgs.msg import DiagnosticArray
            src = tc.source or "gnss"
            cb = lambda msg, s=src, tp=tc.topic: self._on_diagnostic(s, msg, node, tp)
            node.create_subscription(DiagnosticArray, tc.topic, cb, qos)
        elif tc.kind == "rate":
            from gnss_msgs.msg import RawStream
            name = tc.name or "corr"
            cb = lambda msg, n=name, tp=tc.topic: self._on_rate(n, msg, node, tp)
            node.create_subscription(RawStream, tc.topic, cb, qos)
        elif tc.kind == "gga":
            from nmea_msgs.msg import Sentence
            node.create_subscription(Sentence, tc.topic,
                                     lambda m: self._dispatch_epoch("gga", m), qos)
        elif tc.kind == "navsat":
            from sensor_msgs.msg import NavSatFix
            node.create_subscription(NavSatFix, tc.topic,
                                     lambda m: self._dispatch_epoch("navsat", m), qos)
        elif tc.kind == "bestpos":
            from novatel_oem7_msgs.msg import BESTPOS
            node.create_subscription(BESTPOS, tc.topic,
                                     lambda m: self._dispatch_epoch("bestpos", m), qos)
        else:
            raise ValueError(f"unknown topic kind: {tc.kind}")

    # --- handlers (rclpy thread) ------------------------------------------
    def _on_rtkfix(self, src: str, msg, node, topic: str) -> None:
        self._node = node
        t = self._now(node)
        st = rtkfix_state(msg, t)
        if st is None:
            return
        with self._lock:
            self._snapshot["clock_t"] = t
            self._snapshot["topics"][topic] = t
            self._snapshot["sources"][src] = st

    def _on_stat(self, msg, node, topic: str) -> None:
        t = self._now(node)
        lines = self._line_buf.feed(msg.data)
        with self._lock:
            for line in lines:
                self._stat_acc.feed(line)
            self._snapshot["clock_t"] = t
            self._snapshot["topics"][topic] = t
            self._snapshot["stat"] = {
                "sats": list(self._stat_acc.sats),
                "tow": getattr(self._stat_acc, "_tow", None),
                "t": t,
            }

    def _on_diagnostic(self, src: str, msg, node, topic: str) -> None:
        t = self._now(node)
        st = diag_state(msg, t)
        with self._lock:
            self._snapshot["clock_t"] = t
            self._snapshot["topics"][topic] = t
            self._snapshot["diagnostics"][src] = st

    def _on_rate(self, name: str, msg, node, topic: str) -> None:
        t = self._now(node)
        n = len(msg.data)
        hits = self._rate_hits.setdefault(name, [])
        hits.append((t, n))
        cutoff = t - _RATE_WINDOW_S
        while hits and hits[0][0] < cutoff:
            hits.pop(0)
        span = (hits[-1][0] - hits[0][0]) if len(hits) > 1 else 0.0
        rate = (sum(b for _, b in hits) / span) if span > 0 else 0.0
        with self._lock:
            self._snapshot["clock_t"] = t
            self._snapshot["topics"][topic] = t
            self._snapshot["rates"][name] = {
                "t": t, "bytes": n, "rate_bps": rate, "last_t": t, "present": True,
            }

    def _dispatch_epoch(self, kind: str, msg) -> None:
        try:
            if kind == "gga":
                ep = _epoch_from_gga(msg)
            elif kind == "navsat":
                ep = _epoch_from_navsat(msg)
            else:
                ep = _epoch_from_bestpos(msg)
        except Exception:
            return
        if ep is None or self._loop is None:
            return
        self._loop.call_soon_threadsafe(self._on_epoch, ep)


# --- legacy converters (pure functions) -----------------------------------
def _epoch_from_gga(msg) -> Epoch | None:
    import time
    parts = msg.sentence.strip().split(",")
    if len(parts) < 10 or not parts[0].endswith("GGA"):
        return None
    try:
        q = int(parts[6]) if parts[6] else None
        sats = int(parts[7]) if parts[7] else None
        lat = _nmea_deg(parts[2], parts[3])
        lon = _nmea_deg(parts[4], parts[5])
        alt = float(parts[9]) if parts[9] else None
        age = float(parts[13]) if len(parts) > 13 and parts[13] else None
    except (ValueError, IndexError):
        return None
    return Epoch(t=time.time(), src="ros_gga", q=q, sats=sats, age=age,
                 lat=lat, lon=lon, alt=alt)


def _nmea_deg(raw: str, hemi: str) -> float | None:
    if not raw:
        return None
    dot = raw.find(".")
    deg = float(raw[: dot - 2])
    minutes = float(raw[dot - 2:])
    val = deg + minutes / 60.0
    return -val if hemi in ("S", "W") else val


def _epoch_from_navsat(msg) -> Epoch | None:
    import time
    if msg.status.status < 0:
        return Epoch(t=time.time(), src="ros_navsat", q=msg.status.status)
    cov = msg.position_covariance
    sde = math.sqrt(cov[0]) if cov[0] > 0 else None
    sdn = math.sqrt(cov[4]) if cov[4] > 0 else None
    sdu = math.sqrt(cov[8]) if cov[8] > 0 else None
    return Epoch(t=time.time(), src="ros_navsat", q=msg.status.status,
                 lat=msg.latitude, lon=msg.longitude, alt=msg.altitude,
                 sde=sde, sdn=sdn, sdu=sdu)


def _epoch_from_bestpos(msg) -> Epoch | None:
    import time
    return Epoch(t=time.time(), src="ros_bestpos",
                 q=int(msg.pos_type.type), sats=int(msg.num_sol_svs),
                 age=float(msg.diff_age),
                 lat=float(msg.lat), lon=float(msg.lon), alt=float(msg.hgt),
                 sde=float(msg.lat_stdev), sdn=float(msg.lon_stdev),
                 sdu=float(msg.hgt_stdev))
