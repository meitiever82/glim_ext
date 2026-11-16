"""Wire config -> collectors -> writers/reserver/events. Entry point of Plan 1/2.

CAN channel naming: "can0" opens SocketCAN; "virtual:<name>" opens a python-can
virtual bus (tests / replay without hardware).
"""
from __future__ import annotations

import asyncio
import json
import logging
import re
import socket
import time
from dataclasses import asdict
from typing import Callable, Coroutine

import uvicorn

from gnss_hmi.api import create_api
from gnss_hmi.broadcast import Broadcaster
from gnss_hmi.config import Config, load_config
from gnss_hmi.collectors.reserve import LocalReserver
from gnss_hmi.collectors.tcp import TcpCollector
from gnss_hmi.diagnosis.base_station import BaseStationMonitor
from gnss_hmi.geo import horiz_dist_m
from gnss_hmi.diagnosis.events import EventMachine
from gnss_hmi.diagnosis.rules import DiagnosisInput, diagnose
from gnss_hmi.parsers.cgi610_can import Cgi610Assembler
from gnss_hmi.parsers.gpchc import LineFramer, parse_gpchc
from gnss_hmi.parsers.rtcm import RtcmFramer, parse_1005
from gnss_hmi.parsers.rtksol import parse_llh_solution
from gnss_hmi.parsers.rtkstat import SatStat, SlipWindow, StatEpochAccumulator
from gnss_hmi.publisher import UdpPublisher
from gnss_hmi.ros_bridge import RosBridge, diag_values, worst_of
from gnss_hmi.solver.rtkrcv import RtkrcvManager
from gnss_hmi.storage.canlog import CandumpWriter
from gnss_hmi.storage.cleanup import cleanup_logs
from gnss_hmi.storage.epochs import Epoch, EpochStore
from gnss_hmi.storage.events import EventStore, MemoryEventStore
from gnss_hmi.storage.rawlog import RawLogWriter
from gnss_hmi.tiles import TileManager, TileStore, describe_store, open_tile_store

_CLEANUP_INTERVAL_S = 3600.0
_SUPERVISE_RESTART_S = 5.0
_DIAGNOSIS_INTERVAL_S = 1.0
_STATUS_INTERVAL_S = 1.0

_logger = logging.getLogger(__name__)


class _LazyCanModule:
    """python-can 惰性代理。

    P0 只读环境不需要 python-can，因此不能在模块导入时就 import 失败；
    同时旧测试会 monkeypatch `main.can.Bus`，所以保留模块级 `can` 名称。
    """

    def __getattr__(self, name):
        import can as _can
        return getattr(_can, name)


can = _LazyCanModule()


class _NullEpochs:
    """storage.enabled=false 时的空实现：保持 replay/report 的调用面。"""

    def add(self, e) -> None:
        return None

    def latest(self, src):
        return None

    def query_last(self, src, t0: float, t1: float, limit: int):
        return []

    def base_history(self):
        return []

    def prune(self, before_t: float) -> int:
        return 0

    def checkpoint(self) -> None:
        return None

    def close(self) -> None:
        return None


class _NullLog:
    def append(self, *a, **k) -> None:
        return None

    def close(self) -> None:
        return None


def _pick_free_port() -> int:
    """Bind to port 0 to let the OS choose a free TCP port, then release it."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
    finally:
        s.close()


class App:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        cfg.data_root.mkdir(parents=True, exist_ok=True)
        self.storage_enabled = bool(cfg.storage.enabled)
        self._started_at = time.time()

        # P0 只读模式不落盘：事件用进程内表、历元用空实现。
        if self.storage_enabled:
            self.events = EventStore(cfg.db_path)
            self.epochs = EpochStore(cfg.db_path)
        else:
            self.events = MemoryEventStore()
            self.epochs = _NullEpochs()

        # 旧自采集路径（TCP/CAN/rtkrcv/stat 文件）只有显式启用且允许存储时才起。
        self.legacy_enabled = bool(self.storage_enabled and (
            cfg.corrections.enabled or cfg.raw_obs.enabled or cfg.gnss_solution.enabled
            or cfg.can_enabled or bool(cfg.rtkrcv.binary)))
        if not self.legacy_enabled:
            _logger.info("只读模式：已禁用 TCP/CAN 自采集与 rtkrcv 子进程")

        self.corr_reserver = LocalReserver()
        self.obs_reserver = LocalReserver()

        self._corr_log = (RawLogWriter(cfg.data_root, "corr", ext="rtcm3")
                          if self.storage_enabled else _NullLog())
        self._obs_log = (RawLogWriter(cfg.data_root, "obs", ext="rtcm3")
                         if self.storage_enabled else _NullLog())
        self._sol_log = (RawLogWriter(cfg.data_root, "sol", ext="txt")
                         if self.storage_enabled else _NullLog())
        self._corr_framer = RtcmFramer()
        self._obs_framer = RtcmFramer()
        self._gpchc_framer = LineFramer()
        self._rtksol_framer = LineFramer()
        self._can_assembler = Cgi610Assembler()

        # Plan 2: epoch store, diagnosis engine, base-station monitor, publisher.
        self.base_monitor = (BaseStationMonitor(self.epochs)
                             if self.legacy_enabled else None)
        self.publisher: UdpPublisher | None = (
            UdpPublisher(cfg.publish.host, cfg.publish.port) if cfg.publish.enabled else None)
        self.event_machine = EventMachine(
            self.events, close_hysteresis_s=cfg.diagnosis.close_hysteresis_s,
            on_transition=self._on_diagnosis_transition)

        # Plan 3a: broadcaster for WebSocket subscribers.
        self.broadcaster = Broadcaster()
        self.last_status: dict | None = None
        # 底图：web.tiles_path（P1 矿区 mbtiles）优先，否则用 tiles 配置的
        # 在线预热缓存 current.mbtiles。二进制缺失/损坏都只影响底图，不影响数据面。
        self.tile_manager = TileManager(cfg.tiles) if cfg.tiles.enabled else None
        self.tile_store: TileStore | None = None
        self.tile_info: dict = {"available": False}
        self._init_tiles()
        # Built lazily in run_forever() -- constructing it here would bind
        # cfg.web.port immediately, which tests that build an App without
        # running it (e.g. test_main_supervision.py) don't want.
        self._web_server: uvicorn.Server | None = None

        # Diagnosis-loop state.
        self._corr_last_t: float | None = None
        self._base_offset: float | None = None
        self._sol = None
        self._sol_t: float | None = None
        self._div_since: float | None = None
        self._slips = SlipWindow()
        # Latest epoch's per-satellite data, tailed from rtkrcv's -r 2
        # solution-status (.stat) file. _latest_sats are skyplot dicts
        # ({sat,az,el,snr,used}) surfaced via status.sol.sats_json;
        # _latest_satstats are the SatStat objects the diagnosis multipath
        # rule consumes.
        self._latest_sats: list[dict] = []
        self._latest_satstats: list[SatStat] = []
        # control points as (lat, lon) for the diagnosis abs-ref check; immutable
        # config, so built once here rather than per 1 Hz tick
        self._control_points_ll = [(cp.lat, cp.lon) for cp in cfg.control_points]
        self._last_epoch_write: dict[str, int] = {}
        self._last_pos_pub: float = 0.0

        # Legacy collectors (only when explicitly enabled).
        self._collectors: list[TcpCollector] = []
        self._sol_collector = None
        self._bus = None
        self._can_collector = None
        self._can_log = None
        self._rtkrcv_manager: RtkrcvManager | None = None
        self._rtkrcv_sol_collector: TcpCollector | None = None
        self._sol_port: int | None = None
        if self.legacy_enabled:
            self._init_legacy_collectors()

        # ROS 只读桥（唯一数据源）。
        self.bridge: RosBridge | None = RosBridge(cfg.ros) if cfg.ros.enabled else None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._open_ros_events: dict[str, dict] = {}

    def _init_legacy_collectors(self) -> None:
        """rtk-monitor 基线的自采集路径，仅在旧配置显式启用时构造。"""
        cfg = self.cfg
        self._sol_collector = TcpCollector(
            "gnss_solution_link", cfg.gnss_solution.host, cfg.gnss_solution.port,
            self._on_sol, self._on_event, listen=cfg.gnss_solution.listen)
        self._collectors = [
            TcpCollector("corrections_link", cfg.corrections.host, cfg.corrections.port,
                         self._on_corr, self._on_event, listen=cfg.corrections.listen),
            TcpCollector("raw_obs_link", cfg.raw_obs.host, cfg.raw_obs.port,
                         self._on_obs, self._on_event, listen=cfg.raw_obs.listen),
            self._sol_collector,
        ]
        # python-can 只在旧路径需要时才导入，P0 运行环境无需安装。
        from gnss_hmi.collectors.can import CanCollector

        channel = cfg.can_channel
        bus_factory: Callable[[], can.BusABC] | None = None
        if channel.startswith("virtual:"):
            name = channel.split(":", 1)[1]
            self._bus = can.Bus(interface="virtual", channel=name)
            # Name the candump log stream so it matches the "can*_*.log" glob
            # used by tools and tests (e.g. virtual:e2e -> "can_e2e").
            log_name = "can_" + name
        else:
            self._bus = can.Bus(interface="socketcan", channel=channel)
            log_name = channel
            # Real hardware: wire the reopen watchdog so a wedged SocketCAN
            # link (e.g. after a controller reset) gets a fresh bus instead
            # of leaving the collector reading from a dead one forever.
            bus_factory = lambda: can.Bus(interface="socketcan", channel=channel)
        self._can_log = CandumpWriter(cfg.data_root, log_name)
        self._can_collector = CanCollector(self._bus, self._on_can, self._on_event,
                                            bus_factory=bus_factory)

        # rtkrcv solver: only wired when a binary path is configured. The
        # corr/obs ports it dials come from the reserver's bound ports, which
        # aren't known until the reservers start (Task-4 auto-bind: config
        # port 0 means "pick a free port"), so RtkrcvManager itself is built
        # in run_forever(). The sol port, however, is resolved here in
        # __init__ per the binding note: it must be known (and stable) before
        # run_forever so sol_collector_port()-style callers can rely on it.
        if cfg.rtkrcv.binary:
            self._sol_port = cfg.rtkrcv.sol_port or _pick_free_port()
            self._rtkrcv_sol_collector = TcpCollector(
                "rtkrcv_sol", "127.0.0.1", self._sol_port,
                self._on_rtksol, self._on_event)


    # --- stream callbacks: log first, then fan out -------------------------
    # Every callback below is invoked synchronously from inside a collector's
    # pump loop. None of them may let an exception escape -- a storage error
    # (e.g. sqlite3.OperationalError, a full disk) must never propagate back
    # into a collector and take down its route (spec: "collection never
    # stops"). Append and broadcast are guarded in separate try/except blocks
    # so a failed append does not skip the broadcast (spec Sec3.3).
    def _on_corr(self, data: bytes, t: float) -> None:
        # Any corrections traffic (not just 1005/1006) counts as "link alive"
        # for the diagnosis loop's outage rule.
        self._corr_last_t = time.time()
        try:
            for msg in self._corr_framer.feed(data):
                try:
                    self._corr_log.append(msg.raw, msg.msg_type)
                except Exception:
                    _logger.exception("corr rawlog append failed")
                if msg.msg_type in (1005, 1006):
                    if self.base_monitor is None:
                        continue
                    try:
                        x, y, z = parse_1005(msg.payload)
                        self._base_offset = self.base_monitor.feed(time.time(), x, y, z)
                    except Exception:
                        _logger.exception("base station update failed")
        except Exception:
            _logger.exception("corr framer feed failed")
        try:
            self.corr_reserver.broadcast(data)
        except Exception:
            _logger.exception("corr broadcast failed")

    def _on_obs(self, data: bytes, t: float) -> None:
        try:
            for msg in self._obs_framer.feed(data):
                self._obs_log.append(msg.raw, msg.msg_type)
        except Exception:
            _logger.exception("obs rawlog append failed")
        try:
            self.obs_reserver.broadcast(data)
        except Exception:
            _logger.exception("obs broadcast failed")

    def _on_sol(self, data: bytes, t: float) -> None:
        # Route 3: GPCHC ASCII sentences from the CGI-610's TCP interface.
        try:
            self._sol_log.append(data)
        except Exception:
            _logger.exception("sol rawlog append failed")
        try:
            for line in self._gpchc_framer.feed(data):
                e = parse_gpchc(line)
                if e is None:
                    continue
                sec = int(t)
                if self._last_epoch_write.get("gpchc") == sec:
                    continue
                self._last_epoch_write["gpchc"] = sec
                self.epochs.add(Epoch(
                    t=t, src="gpchc", q=e.sat_status, sats=e.nsv1, age=e.diff_age,
                    lat=e.lat, lon=e.lon, alt=e.alt, heading=e.heading, speed=e.speed))
        except Exception:
            _logger.exception("gpchc epoch decode failed")

    def _on_rtksol(self, data: bytes, t: float) -> None:
        # rtkrcv's own llh-format solution stream (only wired when enabled).
        try:
            for line in self._rtksol_framer.feed(data):
                sol = parse_llh_solution(line)
                if sol is None:
                    continue
                self._sol = sol
                self._sol_t = t
                # rtkrcv emits at the solver's full solution rate (up to
                # ~5-10Hz); decimate epoch storage to 1Hz like the other
                # sources (gpchc/can) to avoid unbounded DB growth over a
                # multi-week unattended run. publish_fix stays per-solution.
                sec = int(t)
                if self._last_epoch_write.get("rtkrcv") != sec:
                    self._last_epoch_write["rtkrcv"] = sec
                    try:
                        self.epochs.add(Epoch(
                            t=t, src="rtkrcv", q=sol.q, sats=sol.ns, age=sol.age,
                            lat=sol.lat, lon=sol.lon, alt=sol.alt,
                            sde=sol.sde, sdn=sol.sdn, sdu=sol.sdu, ratio=sol.ratio,
                            # persist the current epoch's sky data so replay can
                            # reconstruct the skyplot (status.sol.sats_json)
                            sats_json=json.dumps(self._latest_sats)))
                    except Exception:
                        _logger.exception("rtkrcv epoch store failed")
                if self.publisher is not None:
                    try:
                        self.publisher.publish_fix(sol, heading=self._latest_heading(),
                                                   host_time=self._sol_t)
                    except Exception:
                        _logger.exception("publish_fix failed")
        except Exception:
            _logger.exception("rtksol decode failed")

    def _latest_heading(self) -> float | None:
        """Most recent heading from either the gpchc or can epoch source."""
        g = self.epochs.latest("gpchc")
        c = self.epochs.latest("can")
        if g is None:
            return c.heading if c is not None else None
        if c is None:
            return g.heading
        return g.heading if g.t >= c.t else c.heading

    def _on_can(self, can_id: int, data: bytes, t: float) -> None:
        try:
            self._can_log.append(can_id, data, t)
        except Exception:
            _logger.exception("can log append failed")
        try:
            cyc = self._can_assembler.feed(can_id, data, t)
            if cyc is not None:
                sec = int(cyc.host_time)
                if self._last_epoch_write.get("can") != sec:
                    self._last_epoch_write["can"] = sec
                    self.epochs.add(Epoch(
                        t=cyc.host_time, src="can", q=cyc.sat_status, sats=cyc.sats_used,
                        age=cyc.diff_age, lat=cyc.lat, lon=cyc.lon, alt=cyc.alt,
                        sde=cyc.pos_sigma[0], sdn=cyc.pos_sigma[1], sdu=cyc.pos_sigma[2],
                        heading=cyc.heading, speed=cyc.vel[3]))
                # Publish position message at ≤5Hz (independent of 1Hz epoch decimation).
                if cyc.host_time - self._last_pos_pub >= 0.2:
                    self._last_pos_pub = cyc.host_time
                    try:
                        self.broadcaster.publish({
                            "type": "position",
                            "t": cyc.host_time,
                            "src": "can",
                            "lat": cyc.lat,
                            "lon": cyc.lon,
                            "heading": cyc.heading,
                            "q": cyc.sat_status,
                            "speed": cyc.vel[3],
                            "replay": False   # live; replay.py sets True
                        })
                    except Exception:
                        _logger.exception("position broadcast failed")
        except Exception:
            _logger.exception("can epoch decode failed")

    def _on_event(self, name: str, state: str, detail: str) -> None:
        try:
            self.events.record(time.time(), name, state, detail)
        except Exception:
            _logger.exception("event record failed: %s %s %r", name, state, detail)

    def _on_diagnosis_transition(self, kind: str, verdict, t: float) -> None:
        if self.publisher is not None:
            try:
                self.publisher.publish_event(kind, verdict, t)
            except Exception:
                _logger.exception("publish_event failed")
        # Publish event message to broadcaster regardless of UDP publisher state.
        try:
            self.broadcaster.publish({
                "type": "event",
                "t": t,
                "action": kind,
                "replay": False,   # live; replay.py sets True
                "event": {
                    "t": t,
                    "level": verdict.level,
                    "code": verdict.code,
                    "message": verdict.message
                }
            })
        except Exception:
            _logger.exception("event broadcast failed")

    def sol_collector_port(self) -> int | None:
        """Route-3 (GPCHC) listen port. Used by tests and Plan 3's UI."""
        return self._sol_collector.bound_port if self._sol_collector is not None else None

    def web_port(self) -> int | None:
        """Bound port of the FastAPI/uvicorn web server, or None before it
        has started listening (e.g. before run_forever, or mid-startup)."""
        if self._web_server is None:
            return None
        try:
            return self._web_server.servers[0].sockets[0].getsockname()[1]
        except (IndexError, AttributeError):
            return None

    def _init_tiles(self) -> None:
        cfg = self.cfg
        path = cfg.web.tiles_path
        if not path and self.tile_manager is not None:
            path = self.tile_manager.current_path() or ""
        self.tile_store = open_tile_store(path) if path else None
        if self.tile_store is None:
            self.tile_info = {"available": False,
                              "attribution": cfg.tiles.attribution}
            _logger.info("底图不可用（path=%r）：页面退网格/在线底图", path)
            return
        self.tile_info = (describe_store(path, cfg.tiles)
                          if path == cfg.web.tiles_path else self.tile_manager.info())

    async def _tiles_warm(self) -> None:
        """启动预热：联网失败只记日志，绝不影响只读数据链路。"""
        try:
            res = await asyncio.to_thread(self.tile_manager.sync)
            _logger.info("底图预热结果：%s", res)
            if res.get("ok") and not res.get("unchanged"):
                self._init_tiles()
        except Exception:
            _logger.exception("底图预热失败（不影响数据面）")

    # --- lifecycle ---------------------------------------------------------
    async def run_forever(self) -> None:
        self._loop = asyncio.get_running_loop()
        supervised: list[tuple[str, Callable[[], Coroutine]]] = []
        if self.legacy_enabled:
            await self.corr_reserver.start(self.cfg.reserve_corrections_port)
            await self.obs_reserver.start(self.cfg.reserve_raw_obs_port)
            if self.cfg.rtkrcv.binary:
                # corr/obs ports dial the reservers' fan-out (bound ports are only
                # known now, after reserve_*_port=0 auto-bind has resolved).
                self._rtkrcv_manager = RtkrcvManager(
                    self.cfg.rtkrcv.binary, run_dir=self.cfg.data_root / "rtkrcv",
                    corr_port=self.corr_reserver.bound_port,
                    obs_port=self.obs_reserver.bound_port,
                    sol_port=self._sol_port,
                    extra_args=self.cfg.rtkrcv.extra_args, on_event=self._on_event)
            supervised.extend((c.name, c.run) for c in self._collectors)
            if self._can_collector is not None:
                supervised.append(("can_collector", self._can_collector.run))
            supervised.append(("diagnosis_loop", self._diagnosis_loop))
            if self._rtkrcv_manager is not None:
                supervised.append(("rtkrcv", self._rtkrcv_manager.run))
                supervised.append(("rtkstat", self._stat_reader))
            if self._rtkrcv_sol_collector is not None:
                supervised.append((self._rtkrcv_sol_collector.name,
                                   self._rtkrcv_sol_collector.run))
        if self.publisher is not None:
            await self.publisher.start()
        if self.bridge is not None:
            self.bridge.set_position_callback(self._on_ros_position)
            if self.bridge.start():
                supervised.append(("ros_status", self._ros_status_loop))
            else:
                _logger.error("ROS 桥启动失败，HMI 将只提供空页面：%s", self.bridge.error)
        self._web_server = uvicorn.Server(
            uvicorn.Config(create_api(self), host=self.cfg.web.host, port=self.cfg.web.port,
                           log_level="warning", timeout_graceful_shutdown=5))
        supervised.append(("web", self._web_server.serve))
        tasks = [asyncio.create_task(self._supervise(name, factory))
                 for name, factory in supervised]
        tasks.append(asyncio.create_task(self._cleanup_loop()))
        if self.tile_manager is not None and self.cfg.tiles.warm_on_start:
            tasks.append(asyncio.create_task(self._tiles_warm()))
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            for t in tasks:
                t.cancel()
            raise

    # --- ROS 只读状态聚合 ---------------------------------------------------
    def _on_ros_position(self, msg: dict) -> None:
        """ROS 线程回调 → 事件循环线程广播（asyncio.Queue 非线程安全）。"""
        loop = self._loop
        if loop is None:
            return
        try:
            loop.call_soon_threadsafe(self.broadcaster.publish, msg)
        except RuntimeError:
            pass           # 事件循环已关闭

    async def _ros_status_loop(self) -> None:
        while True:
            await asyncio.sleep(_STATUS_INTERVAL_S)
            try:
                self._ros_tick()
            except Exception:
                _logger.exception("ROS status tick 失败")

    def _ros_tick(self) -> None:
        """按 1 Hz 聚合 ROS 快照 → status 消息（只读，不重算诊断）。"""
        assert self.bridge is not None
        now = time.time()
        snap = self.bridge.snapshot(now)
        stale_s = self.cfg.ros.stale_s
        sources = snap["sources"]
        for src in sources.values():
            fresh = (src.get("t_recv") is not None
                     and (now - src["t_recv"]) <= stale_s)
            src["available"] = bool(fresh)
            if not fresh:
                src["reason"] = "无数据" if src.get("t_recv") is None else "数据中断"

        diag_gnss = snap["diag_gnss"]
        diag_rtkrcv = snap["diag_rtkrcv"]
        startup = (now - self._started_at) < self.cfg.ros.startup_grace_s

        def _diag_fresh(d):
            return d is not None and (now - float(d.get("t_recv") or 0.0)) <= stale_s

        # 诊断源本身也会断流：超时后不能继续把旧快照当“当前状态”，
        # 否则横幅会永远冻结在最后一条 WARN 上。
        stale_diags = [name for name, d in (("/gnss/diagnostics", diag_gnss),
                                            ("/rtkrcv_node/diagnostics", diag_rtkrcv))
                       if d is not None and not _diag_fresh(d)]
        worst = worst_of(diag_gnss if _diag_fresh(diag_gnss) else None,
                         diag_rtkrcv if _diag_fresh(diag_rtkrcv) else None)
        vals = diag_values(diag_gnss)
        level = (worst["level_name"].lower() if worst else "ok")
        verdict = {
            "level": level,
            "code": vals["status_code"] or "",
            "message": worst["message"] if worst else "",
            "open_events": vals["open_events"],
            "divergence_m": vals["divergence_m"],
            "verdicts": vals["verdicts"],
            "startup_grace": vals["startup_grace"],
        }

        rates = snap["rates"]
        def _rate(topic: str):
            return rates.get(topic, (0.0, None))
        rtcm_bps, rtcm_idle = _rate("/gnss/rtcm_corrections")
        obs_bps, obs_idle = _rate("/gnss/raw_obs")
        startup = (now - self._started_at) < self.cfg.ros.startup_grace_s
        all_stale = not any(s["available"] for s in sources.values())
        if all_stale and not startup:
            verdict["level"] = "error" if verdict["level"] == "ok" else verdict["level"]
            verdict["message"] = verdict["message"] or "数据中断：检查话题与 QoS"
        elif stale_diags and not startup and verdict["level"] in ("ok", "info"):
            verdict["level"] = "warning"
            verdict["message"] = (f"诊断中断：{'、'.join(stale_diags)} "
                                  f"超过 {stale_s:g} s 无数据")

        status_msg = {
            "type": "status",
            "t": now,
            "replay": False,
            "sources": {"app": sources["app"], "fused": sources["fused"]},
            "verdict": verdict,
            "diagnostics": {"gnss": diag_gnss, "rtkrcv": diag_rtkrcv},
            "corr_rate": {
                "rtcm_bps": rtcm_bps,
                "obs_bps": obs_bps,
                "rtcm_idle_s": rtcm_idle,
                "obs_idle_s": obs_idle,
                "rtcm_stale": rtcm_idle is None or rtcm_idle > stale_s,
                "obs_stale": obs_idle is None or obs_idle > stale_s,
            },
            "satellites": snap["satellites"],
            "sat_count": len(snap["satellites"]),
            "links": self.bridge.link_status(now, self._started_at + self.cfg.ros.startup_grace_s),
            "stale": all_stale,
            "startup": startup,
            "ros_error": snap["ros_error"],
        }
        self.last_status = status_msg
        self.broadcaster.publish(status_msg)
        self._publish_ros_events(verdict, now)

    @staticmethod
    def _split_verdict(raw: str) -> tuple[str, str]:
        """`verdict.<code>` 形如 "WARN|差分中断"（级别|文案），容错纯文案。"""
        if not raw:
            return "warning", ""
        level = "warning"
        text = raw
        m = re.match(r"^\s*(OK|WARN|ERROR|STALE)\b[\s|:：-]*(.*)$", raw)
        if m:
            level = {"OK": "info", "WARN": "warning",
                     "ERROR": "critical", "STALE": "warning"}.get(m.group(1), "warning")
            text = m.group(2).strip()
        return level, text

    def _publish_ros_events(self, verdict: dict, now: float) -> None:
        """把 open_events 变化转成 open/close 事件：本地不重跑规则，只跟踪差集。
        事件同时写入进程内事件表，页面刷新/WS 重连后仍有历史。"""
        current: dict[str, dict] = {}
        verdicts = verdict.get("verdicts") or {}
        for code in verdict.get("open_events") or []:
            level, text = self._split_verdict(str(verdicts.get(code, "")))
            current[code] = {"level": level,
                             "message": text or verdict.get("message") or code}
        for code, prev in list(self._open_ros_events.items()):
            if code not in current:
                self.broadcaster.publish({
                    "type": "event", "t": now, "replay": False, "action": "close",
                    "event": {"t": now, "code": code, "level": prev["level"],
                              "message": "已恢复"},
                })
                try:
                    if prev.get("row_id") is not None:
                        self.events.close_event(prev["row_id"], now)
                except Exception:
                    _logger.exception("event close failed: %s", code)
                del self._open_ros_events[code]
        for code, info in current.items():
            if code in self._open_ros_events:
                continue
            try:
                row_id = self.events.record(now, "ros", "open", info["message"],
                                            level=info["level"], code=code)
            except Exception:
                row_id = None
                _logger.exception("event record failed: %s", code)
            self._open_ros_events[code] = {"level": info["level"], "t": now,
                                           "row_id": row_id}
            self.broadcaster.publish({
                "type": "event", "t": now, "replay": False, "action": "open",
                "event": {"t": now, "code": code, "level": info["level"],
                          "message": info["message"]},
            })


    async def _stat_reader(self) -> None:
        """Tail rtkrcv's -r 2 solution-status file and keep the latest epoch's
        per-satellite sky data in self._latest_sats for the skyplot. rtkrcv
        names the file rtkrcv_<time>.stat in run_dir; pick the actively-written
        one by mtime (the name's timestamp is unreliable), and re-select if
        rtkrcv restarts and opens a fresh file."""
        run_dir = (self.cfg.data_root / "rtkrcv").resolve()
        cur_path = None
        fh = None
        acc = StatEpochAccumulator()
        # slip window is queried on the wall clock (count(now)), so feed wall
        # time, not the $SAT GPST tow
        on_slip = lambda sat, slipc: self._slips.feed(time.time(), sat, slipc)
        try:
            while True:
                try:
                    stats = list(run_dir.glob("rtkrcv_*.stat"))
                    newest = max(stats, key=lambda p: p.stat().st_mtime) if stats else None
                    if newest is not None and newest != cur_path:
                        if fh is not None:
                            fh.close()
                        fh = open(newest, "r")
                        cur_path = newest
                        acc.reset()
                    if fh is not None:
                        while True:
                            pos = fh.tell()
                            line = fh.readline()
                            if not line:
                                break
                            if not line.endswith("\n"):
                                fh.seek(pos)   # partial line mid-write; retry next poll
                                break
                            acc.feed(line, on_slip=on_slip)
                        self._latest_sats = acc.sats
                        self._latest_satstats = acc.satstats
                except Exception:
                    _logger.exception("stat reader iteration failed")
                await asyncio.sleep(0.5)
        finally:
            if fh is not None:
                fh.close()

    async def _diagnosis_loop(self) -> None:
        while True:
            await asyncio.sleep(_DIAGNOSIS_INTERVAL_S)
            try:
                self._diagnosis_tick()
            except Exception:
                # A transient failure (e.g. a bad epoch row) must not kill
                # the 1Hz cadence -- log and try again next tick.
                _logger.exception("diagnosis tick failed")

    def _diagnosis_tick(self) -> None:
        now = time.time()
        # A frozen solver output (rtkrcv stopped emitting) must not be treated
        # as a live solution forever -- gate it on recency so the rule chain
        # degrades to the "no_solution" path instead of reporting a stale fix
        # at a stale position indefinitely.
        sol = (self._sol if (self._sol_t is not None
                              and now - self._sol_t < self.cfg.diagnosis.sol_stale_s)
               else None)
        div_m = None
        can_e = self.epochs.latest("can")
        if can_e is not None and now - can_e.t > self.cfg.diagnosis.sol_stale_s:
            # Same freshness gate as the solver output: latest("can") returns
            # the newest row regardless of age, so after the CAN link dies a
            # minutes-old epoch would keep feeding the corr-age fallback
            # (rule 1 blaming the corrections link for a dead CAN feed) and
            # the event lat/lon fallback (pinning events to a stale position).
            can_e = None
        if (sol is not None and can_e is not None and can_e.lat is not None
                and abs((self._sol_t or 0) - can_e.t) < 2.0):
            div_m = horiz_dist_m(sol.lat, sol.lon, can_e.lat, can_e.lon)
            sigma = max(1e-3, (sol.sdn ** 2 + sol.sde ** 2) ** 0.5)
            if div_m > self.cfg.diagnosis.divergence_sigma * sigma:
                self._div_since = self._div_since or now
            else:
                self._div_since = None
        else:
            # No fresh sol/CAN pair to compare -- any previously-held
            # divergence timer is stale and must not survive to fire rule 7
            # instantly once data returns.
            self._div_since = None
        inp = DiagnosisInput(
            now=now, corr_last_t=self._corr_last_t,
            corr_age=sol.age if sol else
                     (can_e.age if can_e else None),
            base_offset_m=self._base_offset,
            sol=sol, sol_t=self._sol_t,
            sats=self._latest_satstats, slip_count_30s=self._slips.count(now),
            divergence_m=div_m, divergence_since=self._div_since,
            solver_enabled=(self.cfg.rtkrcv.binary != ""),
            control_points=self._control_points_ll)
        v = diagnose(inp, self.cfg.diagnosis)
        lat = sol.lat if sol else (can_e.lat if can_e else None)
        lon = sol.lon if sol else (can_e.lon if can_e else None)
        metrics = {
            "divergence_m": div_m or 0.0,
            "sats_min": float(sol.ns) if sol else 0.0,
            "corr_gap_s": now - self._corr_last_t if self._corr_last_t else 0.0
        }
        self.event_machine.update(now, v, lat, lon, metrics=metrics)

        # Publish status message (1Hz from diagnosis loop).
        try:
            sol_dict = None
            if sol is not None:
                sol_dict = {
                    "t": sol.t,
                    "q": sol.q,
                    "sats": sol.ns,
                    "age": sol.age,
                    "ratio": sol.ratio,
                    "lat": sol.lat,
                    "lon": sol.lon,
                    "alt": sol.alt,
                    "sdn": sol.sdn,
                    "sde": sol.sde,
                    "sdu": sol.sdu,
                    # per-satellite sky data for the skyplot (empty until the
                    # rtkrcv .stat tail has an epoch)
                    "sats_json": json.dumps(self._latest_sats)
                }
            can_dict = asdict(can_e) if can_e is not None else None
            gpchc_e = self.epochs.latest("gpchc")
            gpchc_dict = asdict(gpchc_e) if gpchc_e is not None else None
            status_msg = {
                "type": "status",
                "t": now,
                "replay": False,   # live; replay.py sets True
                "verdict": {
                    "level": v.level,
                    "code": v.code,
                    "message": v.message
                },
                "sol": sol_dict,
                "can": can_dict,
                "gpchc": gpchc_dict,
                "corr": {
                    # null means "never received" -- 0.0 is a legitimate
                    # offset/timestamp value and must stay distinguishable
                    # from it (matches replay.py's status.corr contract).
                    "last_t": self._corr_last_t,
                    "base_offset_m": self._base_offset
                }
            }
            self.last_status = status_msg
            self.broadcaster.publish(status_msg)
        except Exception:
            _logger.exception("status broadcast failed")

    async def _supervise(self, name: str, coro_factory: Callable[[], Coroutine]) -> None:
        """Run coro_factory() forever, restarting it on any non-cancellation
        exception. A single misbehaving route (a collector task dying for an
        unforeseen reason) must never unwind the other routes via gather.
        Catches SystemExit as well as Exception: uvicorn's serve() sys.exit()s
        on bind failure; consequently any supervised coroutine calling sys.exit
        becomes a restart, not a process exit — intentional under the
        collection-never-stops principle."""
        while True:
            try:
                await coro_factory()
            except asyncio.CancelledError:
                raise
            except (Exception, SystemExit):
                # SystemExit is included because uvicorn.Server.serve() (the
                # "web" route) calls sys.exit() itself on a bind failure
                # (e.g. cfg.web.port already in use) instead of raising --
                # left uncaught, that BaseException would unwind past this
                # try/except and crash the whole App via gather(), which the
                # spec's "collection never stops" principle forbids for a
                # web-layer failure just as much as for a collector one.
                _logger.exception("%s crashed; restarting in %.0fs", name,
                                   _SUPERVISE_RESTART_S)
                try:
                    self._on_event(name, "crashed", "restarting")
                except Exception:
                    _logger.exception("failed to record crash event for %s", name)
                await asyncio.sleep(_SUPERVISE_RESTART_S)
                continue
            # coro_factory() returned cleanly (no exception) -- e.g. uvicorn's
            # serve() after should_exit is set. Without a delay here the
            # while loop respins with no await point in between, hot-looping
            # this task and starving the event loop for every other route.
            # The same restart delay as the crash path applies.
            _logger.warning("%s returned; restarting in %.0fs", name,
                             _SUPERVISE_RESTART_S)
            await asyncio.sleep(_SUPERVISE_RESTART_S)

    async def _cleanup_loop(self) -> None:
        while True:
            try:
                if self.storage_enabled:
                    cleanup_logs(self.cfg.data_root, self.cfg.retention_days,
                                 self.cfg.disk_watermark_pct)
                    cutoff = time.time() - self.cfg.db_retention_days * 86400.0
                    self.epochs.prune(cutoff)
                    self.events.prune(cutoff)
                    try:
                        self.epochs.checkpoint()
                    except Exception:
                        _logger.exception("epochs checkpoint failed")
                    try:
                        self.events.checkpoint()
                    except Exception:
                        _logger.exception("events checkpoint failed")
                else:
                    self.events.prune(time.time() - 86400.0)
            except Exception:
                # A transient filesystem error must not kill the hourly loop.
                _logger.exception("cleanup failed")
            await asyncio.sleep(_CLEANUP_INTERVAL_S)

    async def shutdown(self) -> None:
        if self.bridge is not None:
            self.bridge.stop()
        if self._web_server is not None:
            self._web_server.should_exit = True
        for w in (self._corr_log, self._obs_log, self._sol_log):
            w.close()
        if self._can_log is not None:
            self._can_log.close()
        await self.corr_reserver.stop()
        await self.obs_reserver.stop()
        if self._bus is not None:
            self._bus.shutdown()
        if self.publisher is not None:
            await self.publisher.stop()
        # Note: self.events/self.epochs are intentionally left open here. App
        # exposes both as the diagnostic/query surface for callers (and for
        # Plan 2's diagnosis engine host), so they must stay queryable after
        # shutdown() returns. Callers that want to fully release the sqlite
        # handles can call .close() on them once done.


def build_app(cfg: Config) -> App:
    return App(cfg)


def main(argv: list[str] | None = None) -> None:
    import argparse
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    ap = argparse.ArgumentParser(description="gnss_bringup HMI（只读 Web 服务）")
    ap.add_argument("config", nargs="?", default="config.yaml", help="YAML 配置路径")
    ap.add_argument("--config", dest="config_opt", default=None, help="同位置参数")
    args = ap.parse_args(argv)
    cfg = load_config(args.config_opt or args.config)
    app = build_app(cfg)
    try:
        asyncio.run(app.run_forever())
    except KeyboardInterrupt:
        if app.bridge is not None:
            app.bridge.stop()


if __name__ == "__main__":
    main()
