"""只读 ROS 2 适配层：订阅 gnss_bringup 话题，生成 HMI 内存快照。

设计约束（spec 2026-11-02-gnss-web-hmi）：
- 只订阅：不创建 publisher，不调用 service/action，不启动/停止任何节点。
- 单一数据源：诊断以 `/gnss/diagnostics` 为准，本地不重跑规则。
- rclpy 只在 start() 时才 import：纯逻辑（映射/组帧/速率窗口）可在无 ROS
  环境下单测，HMI 的 Web 侧也不因缺少 rclpy 而无法做静态检查。

线程模型：rclpy.spin 在独立线程中运行，回调只写本对象的受锁状态；
主事件循环按 1 Hz 调 snapshot() 取一份拷贝，不跨线程共享消息对象。
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

from gnss_hmi.config import RosCfg, RosTopicCfg
from gnss_hmi.parsers.rtkstat import SatStat, StatEpochAccumulator

_logger = logging.getLogger(__name__)

QUALITY_NONE = 0
QUALITY_SINGLE = 1
QUALITY_DGPS = 2
QUALITY_FLOAT = 3
QUALITY_FIXED = 4

SOURCE_APP = "app"
SOURCE_FUSED = "fused"

_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
_LEVEL_RANK = {"OK": 0, "WARN": 1, "ERROR": 2, "STALE": 3}


def _finite(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _opt(v, keep=lambda x: True):
    """有限且满足 keep 时返回 float，否则 None（前端显示“—”。）"""
    if not _finite(v):
        return None
    f = float(v)
    return f if keep(f) else None


def rtkfix_to_dict(msg, t_recv: float) -> dict | None:
    """`gnss_msgs/RtkFix` → HMI 双源快照。经纬度非有限值直接拒绝（不画假点）。"""
    lat, lon = _opt(msg.latitude), _opt(msg.longitude)
    if lat is None or lon is None:
        return None
    # sigma_enu 是 float64[3]，rclpy 给出 ndarray：不能对它用 `or []`
    # （数组真值判断会抛 ValueError），必须显式判 None。
    raw_sigma = getattr(msg, "sigma_enu", None)
    sigma = [float(x) for x in raw_sigma] if raw_sigma is not None else []
    se, sn, su = (sigma + [float("nan")] * 3)[:3]
    sigma_h = (math.hypot(float(se), float(sn))
               if _finite(se) and _finite(sn) else None)
    stamp = None
    header = getattr(msg, "header", None)
    if header is not None:
        st = getattr(header, "stamp", None)
        if st is not None:
            stamp = float(getattr(st, "sec", 0)) + float(getattr(st, "nanosec", 0)) * 1e-9
    gnss_time = _opt(getattr(msg, "gnss_time", 0.0), lambda x: x > 0)
    latency = (stamp - gnss_time) if (stamp is not None and gnss_time is not None) else None
    quality = int(getattr(msg, "quality", QUALITY_NONE))
    heading_valid = bool(getattr(msg, "heading_valid", False))
    return {
        "available": True,
        "t_recv": t_recv,
        "stamp": stamp,
        "gnss_time": gnss_time,
        "latency_s": latency,
        "quality": quality,
        "raw_status": int(getattr(msg, "raw_status", 0)),
        "lat": lat,
        "lon": lon,
        "alt": _opt(getattr(msg, "altitude", float("nan"))),
        # spec：sigma_enu 为 E/N/U 顺序；水平 σ = hypot(E, N)
        "sigma_e": _opt(se),
        "sigma_n": _opt(sn),
        "sigma_u": _opt(su),
        "sigma_h": sigma_h,
        "age_s": _opt(getattr(msg, "diff_age", float("nan"))),
        "sats": int(getattr(msg, "sats_used", 0)),
        "sats_main": int(getattr(msg, "sats_main", 0)),
        "sats_aux": int(getattr(msg, "sats_aux", 0)),
        "ratio": _opt(getattr(msg, "ratio", 0.0), lambda x: x > 0),
        "heading": _opt(getattr(msg, "heading", float("nan"))) if heading_valid else None,
        "heading_valid": heading_valid,
        "speed": None,           # RtkFix 不带速度，状态栏显示“—”
    }


def unavailable(t_recv: float | None = None, reason: str = "") -> dict:
    return {
        "available": False,
        "t_recv": t_recv,
        "stamp": None, "gnss_time": None, "latency_s": None,
        "quality": None, "raw_status": None,
        "lat": None, "lon": None, "alt": None,
        "sigma_e": None, "sigma_n": None, "sigma_u": None, "sigma_h": None,
        "age_s": None, "sats": None, "sats_main": None, "sats_aux": None,
        "ratio": None, "heading": None, "heading_valid": False, "speed": None,
        "reason": reason,
    }


def _as_int(value, default: int = 0) -> int:
    """rclpy 把 `octet`/`byte` 字段给成 `bytes`（如 DiagnosticStatus.level 是
    b'\\x01'），直接 int() 会抛 ValueError；这里统一收敛成整数。"""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def parse_diagnostic_array(msg, t_recv: float) -> dict:
    """`diagnostic_msgs/DiagnosticArray` → 扁平 dict（供事件/横幅消费）。"""
    statuses = []
    worst = 0
    worst_msg = ""
    for st in getattr(msg, "status", []) or []:
        level = _as_int(getattr(st, "level", 0))
        values = {}
        for kv in getattr(st, "values", []) or []:
            values[str(getattr(kv, "key", ""))] = str(getattr(kv, "value", ""))
        name = str(getattr(st, "name", ""))
        message = str(getattr(st, "message", ""))
        if level > worst:
            worst, worst_msg = level, message
        elif level == worst and not worst_msg:
            worst_msg = message
        statuses.append({
            "name": name,
            "level": level,
            "level_name": _LEVEL_NAMES.get(level, "OK"),
            "message": message,
            "hardware_id": str(getattr(st, "hardware_id", "")),
            "values": values,
        })
    return {"t_recv": t_recv, "level": worst, "level_name": _LEVEL_NAMES.get(worst, "OK"),
            "message": worst_msg, "statuses": statuses}


def worst_of(*diags: dict | None) -> dict | None:
    """多个诊断源取最严重级别；文案保留该源的原文。"""
    present = [d for d in diags if d]
    if not present:
        return None
    worst = max(present, key=lambda d: d.get("level", 0))
    return worst


def diag_values(diag: dict | None) -> dict:
    """从 `gnss_diag` 的 status.values 提取 status_code/open_events/verdict.*。"""
    if not diag:
        return {"status_code": None, "open_events": [], "divergence_m": None,
                "verdicts": {}, "startup_grace": False}
    status = None
    for s in diag.get("statuses", []):
        if s.get("name") == "gnss_diag":
            status = s                 # 取最后一个 gnss_diag（同帧内最新）
    if status is None:
        status = diag["statuses"][-1] if diag.get("statuses") else {"values": {}, "message": ""}
    values = status.get("values") or {}
    open_events = [c for c in (values.get("open_events") or "").split(",") if c]
    verdicts = {k.split(".", 1)[1]: v for k, v in values.items() if k.startswith("verdict.")}
    divergence = _opt(values.get("divergence_m"))
    return {
        "status_code": values.get("status_code") or None,
        "open_events": open_events,
        "divergence_m": divergence,
        "verdicts": verdicts,
        "startup_grace": "启动宽限" in (values.get("status_code") or "")
                        or "启动宽限" in (status.get("message") or ""),
    }


class LineAssembler:
    """把任意分块的字节流重组为完整行；跨块半行保留到下一次 feed。"""

    def __init__(self, max_pending: int = 1 << 16) -> None:
        self._buf = b""
        self._max = max_pending

    def feed(self, data: bytes) -> list[str]:
        if not data:
            return []
        self._buf += data
        if len(self._buf) > self._max and b"\n" not in self._buf:
            # 流里没有换行说明不是行文本（或严重错位），丢弃防止无界增长
            self._buf = b""
            return []
        *lines, self._buf = self._buf.split(b"\n")
        out = []
        for raw in lines:
            try:
                out.append(raw.decode("utf-8", "ignore").rstrip("\r"))
            except Exception:      # decode(errors=ignore) 不会抛，保守兜底
                continue
        return out


@dataclass
class _Rate:
    t: float
    n: int


class RateWindow:
    """滑动窗口字节速率（只统计，不解码）。"""

    def __init__(self, window_s: float = 5.0) -> None:
        self._window = window_s
        self._samples: list[_Rate] = []
        self.last_t: float | None = None

    def add(self, t: float, n: int) -> None:
        self._samples.append(_Rate(t, n))
        self.last_t = t
        self._trim(t)

    def _trim(self, now: float) -> None:
        cutoff = now - self._window
        if len(self._samples) > 256 or (self._samples and self._samples[0].t < cutoff):
            self._samples = [s for s in self._samples if s.t >= cutoff]

    def rate(self, now: float) -> float:
        self._trim(now)
        if not self._samples:
            return 0.0
        total = sum(s.n for s in self._samples)
        # 不除零：窗口内样本跨度至少按 1 s 计，避免单帧突发显示成无限速率。
        span = max(1.0, min(self._window, now - self._samples[0].t))
        return total / span

    def idle_s(self, now: float) -> float | None:
        return None if self.last_t is None else max(0.0, now - self.last_t)


class RosBridge:
    """订阅全部只读话题；snapshot() 返回当前最新状态。"""

    def __init__(self, cfg: RosCfg) -> None:
        self.cfg = cfg
        self.error: str | None = None
        self._lock = threading.Lock()
        self._started = False
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._sources = {SOURCE_APP: unavailable(), SOURCE_FUSED: unavailable()}
        self._diag_gnss: dict | None = None
        self._diag_rtkrcv: dict | None = None
        self._sat_acc = StatEpochAccumulator()
        self._sat_assembler = LineAssembler()
        self._sat_t: float | None = None
        self._rates: dict[str, RateWindow] = {}
        self._last_rx: dict[str, float] = {}
        self._last_pos: dict[str, float] = {}
        self._position_cb = None

    # --- lifecycle ---------------------------------------------------------
    def start(self) -> bool:
        try:
            import rclpy                                    # noqa: PLC0415
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
        except Exception as exc:                            # pragma: no cover
            self.error = f"rclpy 不可用：{exc}"
            _logger.error(self.error)
            return False
        with self._lock:
            if self._started:
                return True
            try:
                rclpy.init(args=None)
            except Exception:
                # 已被外部初始化（同进程集成）时继续
                pass
            node = Node(self.cfg.node_name)
            if self.cfg.use_sim_time:
                node.set_parameters([rclpy.Parameter("use_sim_time", value=True)])
            self._node = node
            for topic in self.cfg.topics:
                if not topic.enabled:
                    continue
                self._subscribe(node, topic)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            self._executor = executor
            self._thread = threading.Thread(target=self._spin, name="gnss-ros", daemon=True)
            self._started = True
            self._thread.start()
        _logger.info("ROS 桥已启动（node=%s，%d 条订阅，只读）",
                     self.cfg.node_name, len([t for t in self.cfg.topics if t.enabled]))
        return True

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:                                   # pragma: no cover
            _logger.exception("ROS 执行器退出")
        finally:
            try:
                self._executor.shutdown()
            except Exception:
                pass

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
            executor, node = self._executor, self._node
            self._executor = self._node = None
        for obj in (executor, node):
            try:
                if obj is executor:
                    obj.shutdown()
                else:
                    obj.destroy_node()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _qos(self, topic: RosTopicCfg):
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
        # auto 用 best_effort：reliable 发布端也能服务 best_effort 订阅，
        # 反过来则不成立，因此这是缺省最稳的选择。
        reliability = (ReliabilityPolicy.RELIABLE if topic.qos == "reliable"
                       else ReliabilityPolicy.BEST_EFFORT)
        return QoSProfile(depth=100, history=QoSHistoryPolicy.KEEP_LAST,
                          reliability=reliability)

    def _subscribe(self, node, topic: RosTopicCfg) -> None:
        qos = self._qos(topic)
        if topic.kind == "rtkfix":
            from gnss_msgs.msg import RtkFix          # noqa: PLC0415
            source = topic.source or SOURCE_APP
            node.create_subscription(
                RtkFix, topic.topic,
                lambda msg, s=source, tp=topic.topic: self._on_rtkfix(s, tp, msg), qos)
        elif topic.kind == "stat":
            from gnss_msgs.msg import RawStream       # noqa: PLC0415
            node.create_subscription(
                RawStream, topic.topic,
                lambda msg, tp=topic.topic: self._on_stat(tp, msg), qos)
        elif topic.kind == "diagnostic":
            from diagnostic_msgs.msg import DiagnosticArray   # noqa: PLC0415
            is_rtkrcv = "rtkrcv" in topic.topic
            node.create_subscription(
                DiagnosticArray, topic.topic,
                lambda msg, r=is_rtkrcv, tp=topic.topic: self._on_diag(r, tp, msg), qos)
        elif topic.kind == "rtcm_rate":
            from gnss_msgs.msg import RawStream       # noqa: PLC0415
            node.create_subscription(
                RawStream, topic.topic,
                lambda msg, n=topic.topic: self._on_bytes(n, msg), qos)
        else:
            _logger.warning("未知订阅类型 kind=%r topic=%s", topic.kind, topic.topic)

    # --- callbacks (ROS 线程) ---------------------------------------------
    def _on_rtkfix(self, source: str, topic: str, msg) -> None:
        now = time.time()
        snap = rtkfix_to_dict(msg, now)
        if snap is None:
            _logger.debug("RtkFix 经纬度非有限值，已忽略")
            return
        with self._lock:
            self._sources[source] = snap
            self._last_rx[topic] = now
            last_pos = self._last_pos.get(source, 0.0)
            emit = now - last_pos >= 0.2        # 位置消息 ≤5 Hz
            if emit:
                self._last_pos[source] = now
        cb = self._position_cb
        if emit and cb is not None and snap["quality"] is not None:
            cb({"type": "position", "t": now, "replay": False, "src": source,
                "lat": snap["lat"], "lon": snap["lon"], "q": snap["quality"],
                "heading": snap["heading"], "speed": None})

    def _on_stat(self, topic: str, msg) -> None:
        now = time.time()
        data = bytes(getattr(msg, "data", b"") or b"")
        lines = self._sat_assembler.feed(data)
        with self._lock:
            for line in lines:
                self._sat_acc.feed(line)
            self._sat_t = now
            self._last_rx[topic] = now

    def _on_diag(self, is_rtkrcv: bool, topic: str, msg) -> None:
        now = time.time()
        parsed = parse_diagnostic_array(msg, now)
        with self._lock:
            if is_rtkrcv:
                self._diag_rtkrcv = parsed
            else:
                self._diag_gnss = parsed
            self._last_rx[topic] = now

    def _on_bytes(self, topic: str, msg) -> None:
        now = time.time()
        n = len(bytes(getattr(msg, "data", b"") or b""))
        with self._lock:
            win = self._rates.get(topic)
            if win is None:
                win = self._rates[topic] = RateWindow()
            win.add(now, n)
            self._last_rx[topic] = now

    def set_position_callback(self, cb) -> None:
        self._position_cb = cb

    # --- snapshot (主线程) -------------------------------------------------
    def snapshot(self, now: float | None = None) -> dict:
        now = time.time() if now is None else now
        with self._lock:
            sources = {k: dict(v) for k, v in self._sources.items()}
            diag_gnss = dict(self._diag_gnss) if self._diag_gnss else None
            diag_rtkrcv = dict(self._diag_rtkrcv) if self._diag_rtkrcv else None
            sats = [dict(s) for s in self._sat_acc.sats]
            satstats = list(self._sat_acc.satstats)
            rates = {k: (w.rate(now), w.idle_s(now)) for k, w in self._rates.items()}
            last_rx = dict(self._last_rx)
        stale_s = self.cfg.stale_s
        for src in sources.values():
            if src.get("available") and src.get("t_recv") is not None:
                src["stale"] = (now - src["t_recv"]) > stale_s
            else:
                src["stale"] = True
        return {
            "sources": sources,
            "satellites": sats,
            "satstats": satstats,
            "sat_t": self._sat_t,
            "diag_gnss": diag_gnss,
            "diag_rtkrcv": diag_rtkrcv,
            "rates": rates,
            "last_rx": last_rx,
            "ros_error": self.error,
        }

    def link_status(self, now: float, grace_end: float) -> list[dict]:
        """每条订阅的存活状态；`grace_end` 之前未收到不算异常。"""
        with self._lock:
            last = dict(self._last_rx)
        out = []
        for t in self.cfg.topics:
            if not t.enabled:
                continue
            lt = last.get(t.topic)
            age = None if lt is None else max(0.0, now - lt)
            if age is None:
                state = "waiting" if now < grace_end else "no_data"
            elif age > self.cfg.stale_s:
                state = "stale"
            else:
                state = "ok"
            out.append({"topic": t.topic, "kind": t.kind, "qos": t.qos,
                        "source": t.source, "age_s": age, "state": state})
        return out

