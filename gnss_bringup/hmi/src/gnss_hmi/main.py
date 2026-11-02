"""Wire config -> collectors -> writers/reserver/events. Entry point of Plan 1/2.

CAN channel naming: "can0" opens SocketCAN; "virtual:<name>" opens a python-can
virtual bus (tests / replay without hardware).
"""
from __future__ import annotations

import asyncio
import json
import logging
import socket
import sys
import time
from dataclasses import asdict, dataclass
from typing import Callable, Coroutine

import can
import uvicorn

from gnss_hmi.api import create_api
from gnss_hmi.broadcast import Broadcaster
from gnss_hmi.config import Config, load_config
from gnss_hmi.collectors.can import CanCollector
from gnss_hmi.collectors.reserve import LocalReserver
from gnss_hmi.collectors.ros_collector import RosCollector, TopicCfg
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
from gnss_hmi.solver.rtkrcv import RtkrcvManager
from gnss_hmi.storage.canlog import CandumpWriter
from gnss_hmi.storage.cleanup import cleanup_logs
from gnss_hmi.storage.epochs import Epoch, EpochStore
from gnss_hmi.storage.events import EventStore
from gnss_hmi.storage.rawlog import RawLogWriter
from gnss_hmi.tiles import TileStore
from gnss_hmi.tiles.manager import resolve_tiles_path

_CLEANUP_INTERVAL_S = 3600.0
_SUPERVISE_RESTART_S = 5.0
_DIAGNOSIS_INTERVAL_S = 1.0
_STALE_S = 3.0
_CORR_OUTAGE_S = 3.0

_logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class StatusVerdict:
    level: str
    code: str
    message: str


def _worse_diag(a: dict | None, b: dict | None) -> dict | None:
    """Pick the most severe of the two diagnostic sources."""
    if a is None:
        return b
    if b is None:
        return a
    return a if a["level"] >= b["level"] else b


def _verdict_from_diag(diag: dict | None) -> StatusVerdict:
    """diagnostic_msgs level -> the frontend's ok/info/warning/serious/critical."""
    if diag is None:
        return StatusVerdict("info", "waiting", "等待诊断数据")
    level = {"ok": "ok", "warning": "warning", "error": "critical",
             "stale": "warning"}.get(diag.get("level_name", "ok"), "ok")
    return StatusVerdict(level, diag.get("status_code") or "-", diag.get("message") or "")


def _event_level(verdict_level: str | None, diag_level: str | None) -> str:
    if verdict_level in ("ok", "info", "warning", "serious", "critical"):
        return verdict_level
    return {"ok": "ok", "warning": "warning", "error": "critical",
            "stale": "warning"}.get(diag_level or "ok", "ok")


def _rate_view(rate: dict | None, now: float) -> dict | None:
    if rate is None:
        return None
    gap = now - rate["last_t"]
    return {"rate_bps": rate["rate_bps"], "last_t": rate["last_t"],
            "gap_s": gap, "outage": gap > _CORR_OUTAGE_S}


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
        self.events = EventStore(cfg.db_path)
        self.corr_reserver = LocalReserver()
        self.obs_reserver = LocalReserver()

        self._corr_log = RawLogWriter(cfg.data_root, "corr", ext="rtcm3")
        self._obs_log = RawLogWriter(cfg.data_root, "obs", ext="rtcm3")
        self._sol_log = RawLogWriter(cfg.data_root, "sol", ext="txt")
        self._corr_framer = RtcmFramer()
        self._obs_framer = RtcmFramer()
        self._gpchc_framer = LineFramer()
        self._rtksol_framer = LineFramer()
        self._can_assembler = Cgi610Assembler()

        # Plan 2: epoch store, diagnosis engine, base-station monitor, publisher.
        self.epochs = EpochStore(cfg.db_path)
        self.base_monitor = BaseStationMonitor(self.epochs)
        self.publisher: UdpPublisher | None = (
            UdpPublisher(cfg.publish.host, cfg.publish.port) if cfg.publish.enabled else None)
        self.event_machine = EventMachine(
            self.events, close_hysteresis_s=cfg.diagnosis.close_hysteresis_s,
            on_transition=self._on_diagnosis_transition)

        # Plan 3a: broadcaster for WebSocket subscribers.
        self.broadcaster = Broadcaster()
        self.last_status: dict | None = None
        # Only serve a cache that passes its SHA-256 sidecar; a truncated
        # current.mbtiles falls back to the newest intact version, then to grid.
        self.tiles_path = resolve_tiles_path(cfg.web.tiles_path)
        try:
            self.tile_store: TileStore | None = TileStore(self.tiles_path or "")
        except FileNotFoundError:
            self.tile_store = None
            _logger.info("tiles not configured (web.tiles_path=%r); /tiles disabled",
                         cfg.web.tiles_path)
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

        # Read-only ROS collector: the single data source for the HMI.
        self._ros_collector: RosCollector | None = None
        self._ros_diag_configured = any(t.kind == "diagnostic" for t in cfg.ros.topics)
        if cfg.ros.enabled and cfg.ros.topics:
            self._ros_collector = RosCollector(
                "ros_link",
                tuple(TopicCfg(topic=t.topic, kind=t.kind, qos=t.qos,
                               source=t.source, name=t.name) for t in cfg.ros.topics),
                self._on_ros_epoch, self._on_event,
                timeout_warn_s=cfg.ros.timeout_warn_s)
        self._ros_open_events: set[str] = set()
        self._ros_clock_t: float | None = None

        # Legacy self-collection routes (TCP/CAN). Disabled in the read-only
        # HMI config; kept only for replay/tests against the old deployment.
        self._sol_collector = (
            TcpCollector("gnss_solution_link", cfg.gnss_solution.host,
                         cfg.gnss_solution.port, self._on_sol, self._on_event,
                         listen=cfg.gnss_solution.listen)
            if cfg.gnss_solution.enabled else None)
        self._collectors: list[TcpCollector] = []
        if cfg.corrections.enabled:
            self._collectors.append(
                TcpCollector("corrections_link", cfg.corrections.host,
                             cfg.corrections.port, self._on_corr, self._on_event,
                             listen=cfg.corrections.listen))
        if cfg.raw_obs.enabled:
            self._collectors.append(
                TcpCollector("raw_obs_link", cfg.raw_obs.host, cfg.raw_obs.port,
                             self._on_obs, self._on_event, listen=cfg.raw_obs.listen))
        if self._sol_collector is not None:
            self._collectors.append(self._sol_collector)

        self._bus = None
        self._can_collector = None
        self._can_log = None
        if cfg.can.enabled:
            channel = cfg.can.channel
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
        self._rtkrcv_manager: RtkrcvManager | None = None
        self._rtkrcv_sol_collector: TcpCollector | None = None
        self._sol_port: int | None = None
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

    def _on_ros_epoch(self, ep: Epoch) -> None:
        # ROS collector entry point: Epochs arrive already parsed and
        # timestamped (marshalled onto this loop via call_soon_threadsafe).
        # Same 1Hz-per-source decimation as the other routes. Storage stays
        # off in the read-only HMI (storage.enabled=false).
        if not self.cfg.storage.enabled:
            return
        try:
            sec = int(ep.t)
            if self._last_epoch_write.get(ep.src) == sec:
                return
            self._last_epoch_write[ep.src] = sec
            self.epochs.add(ep)
        except Exception:
            _logger.exception("ros epoch store failed")

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

    # --- lifecycle ---------------------------------------------------------
    async def run_forever(self) -> None:
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
        if self.publisher is not None:
            await self.publisher.start()
        _logger.info(
            "read-only mode: ros=%s tcp_collectors=%d can=%s solver=%s storage=%s",
            self._ros_collector is not None, len(self._collectors),
            self._can_collector is not None, self.cfg.rtkrcv.binary != "",
            self.cfg.storage.enabled)
        supervised: list[tuple[str, Callable[[], Coroutine]]] = [
            (c.name, c.run) for c in self._collectors
        ]
        if self._can_collector is not None:
            supervised.append(("can_collector", self._can_collector.run))
        supervised.append(("diagnosis_loop", self._diagnosis_loop))
        if self._ros_collector is not None:
            supervised.append((self._ros_collector.name, self._ros_collector.run))
        if self._rtkrcv_manager is not None:
            supervised.append(("rtkrcv", self._rtkrcv_manager.run))
            # Prefer the ROS stat stream ($SAT); only tail the .stat file when
            # no ROS stat topic is configured (otherwise the same $SAT lines
            # would be parsed twice).
            if not any(t.kind == "stat" for t in self.cfg.ros.topics):
                supervised.append(("rtkstat", self._stat_reader))
        if self._rtkrcv_sol_collector is not None:
            supervised.append((self._rtkrcv_sol_collector.name, self._rtkrcv_sol_collector.run))
        self._web_server = uvicorn.Server(
            uvicorn.Config(create_api(self), host=self.cfg.web.host, port=self.cfg.web.port,
                           log_level="warning", timeout_graceful_shutdown=5))
        supervised.append(("web", self._web_server.serve))
        tasks = [asyncio.create_task(self._supervise(name, factory))
                 for name, factory in supervised]
        tasks.append(asyncio.create_task(self._cleanup_loop()))
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            for t in tasks:
                t.cancel()
            raise

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
        snap = self._ros_collector.snapshot() if self._ros_collector is not None else None
        node_now = self._ros_collector.current_time() if self._ros_collector is not None else None
        if snap and snap.get("clock_t"):
            # Follow the node clock so a bag replay (use_sim_time) keeps the
            # page's timestamps aligned with /clock instead of wall time.
            self._ros_clock_t = snap["clock_t"]
        now = node_now or self._ros_clock_t or time.time()

        src_app = snap["sources"]["app"] if snap else None
        src_fused = snap["sources"]["fused"] if snap else None
        stat = (snap or {}).get("stat") or {"sats": [], "tow": None}
        rates = (snap or {}).get("rates") or {"corr": None, "obs": None}

        # Prefer the ROS $SAT stream; the local .stat tail is only a fallback.
        if stat.get("sats"):
            self._latest_sats = stat["sats"]

        diag = None
        if snap:
            diag = _worse_diag(snap["diagnostics"].get("gnss"),
                               snap["diagnostics"].get("rtkrcv"))
        if diag is not None:
            self._emit_ros_events(diag, now)

        if self._ros_diag_configured:
            # /gnss/diagnostics is the single diagnosis source; HMI only formats.
            v = _verdict_from_diag(diag)
            lat = (src_fused or src_app or {}).get("lat")
            lon = (src_fused or src_app or {}).get("lon")
        elif self.cfg.diagnosis.local_fallback:
            v, lat, lon = self._local_diagnosis(now)
        else:
            v, lat, lon = StatusVerdict("ok", "-", ""), None, None

        def _view(st: dict | None) -> dict | None:
            if st is None:
                return None
            out = dict(st)
            out["stale"] = (now - st["t"]) > _STALE_S
            return out

        app_view = _view(src_app)
        fused_view = _view(src_fused)

        # --- legacy status fields (kept for the old frontend + replay contract)
        sol_dict = None
        if src_app is not None:
            sol_dict = {
                "t": src_app["t"], "q": src_app["q"], "sats": src_app["sats"],
                "age": src_app["age"], "ratio": src_app["ratio"],
                "lat": src_app["lat"], "lon": src_app["lon"], "alt": src_app["alt"],
                "sdn": src_app["sigma_n"], "sde": src_app["sigma_e"],
                "sdu": src_app["sigma_u"],
                "sats_json": json.dumps(self._latest_sats),
            }
        else:
            sol = self._sol if (self._sol_t is not None
                                and now - self._sol_t < self.cfg.diagnosis.sol_stale_s) else None
            if sol is not None:
                sol_dict = {
                    "t": sol.t, "q": sol.q, "sats": sol.ns, "age": sol.age,
                    "ratio": sol.ratio, "lat": sol.lat, "lon": sol.lon, "alt": sol.alt,
                    "sdn": sol.sdn, "sde": sol.sde, "sdu": sol.sdu,
                    "sats_json": json.dumps(self._latest_sats),
                }
        can_e = self.epochs.latest("can")
        can_dict = asdict(can_e) if can_e is not None else None
        if can_dict is None and src_fused is not None:
            # ROS-only deployment: expose the fused solution under the legacy
            # "can" key so old clients still render a position.
            can_dict = {
                "t": src_fused["t"], "src": "ros_fused", "q": src_fused["q"],
                "sats": src_fused["sats"], "age": src_fused["age"],
                "lat": src_fused["lat"], "lon": src_fused["lon"], "alt": src_fused["alt"],
                "sde": src_fused["sigma_e"], "sdn": src_fused["sigma_n"],
                "sdu": src_fused["sigma_u"], "ratio": src_fused["ratio"],
                "heading": src_fused["heading"], "speed": None, "sats_json": None,
            }
        gpchc_e = self.epochs.latest("gpchc")
        gpchc_dict = asdict(gpchc_e) if gpchc_e is not None else None

        corr_rate = _rate_view(rates.get("corr"), now)
        obs_rate = _rate_view(rates.get("obs"), now)
        link = {
            "corr": corr_rate,
            "obs": obs_rate,
            "rtkrcv": (snap or {}).get("diagnostics", {}).get("rtkrcv") and {
                "level": snap["diagnostics"]["rtkrcv"]["level_name"],
                "message": snap["diagnostics"]["rtkrcv"]["message"],
            },
            "topics": (snap or {}).get("topics", {}),
        }
        stale = all(x is None or x["stale"] for x in (app_view, fused_view))

        status_msg = {
            "type": "status",
            "t": now,
            "replay": False,   # live; replay.py sets True
            "stale": stale,
            "verdict": {"level": v.level, "code": v.code, "message": v.message},
            "sources": {"app": app_view, "fused": fused_view},
            "sky": json.dumps(self._latest_sats),
            "corr_rate": corr_rate,
            "link": link,
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
        try:
            self.broadcaster.publish(status_msg)
        except Exception:
            _logger.exception("status broadcast failed")

    def _local_diagnosis(self, now: float) -> tuple["StatusVerdict", float | None, float | None]:
        """Legacy rule chain -- only reached when no ROS diagnostics topic is
        configured and diagnosis.local_fallback is explicitly enabled."""
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
        if lat is None:
            # ROS-fed epochs (bynav etc.) as last position fallback so events
            # still get pinned to a location on ROS-only deployments.
            ros_e = self.epochs.latest("ros_bestpos") or self.epochs.latest("ros_navsat")
            if ros_e is not None and now - ros_e.t < self.cfg.diagnosis.sol_stale_s:
                lat, lon = ros_e.lat, ros_e.lon
        metrics = {
            "divergence_m": div_m or 0.0,
            "sats_min": float(sol.ns) if sol else 0.0,
            "corr_gap_s": now - self._corr_last_t if self._corr_last_t else 0.0
        }
        self.event_machine.update(now, v, lat, lon, metrics=metrics)
        return StatusVerdict(v.level, v.code, v.message), lat, lon

    def _emit_ros_events(self, diag: dict, now: float) -> None:
        """Turn open_events changes into open/close WS events (no local rules)."""
        cur = set(diag.get("open_events") or [])
        verdicts = diag.get("verdict") or {}
        for code in sorted(cur - self._ros_open_events):
            info = verdicts.get(code) or {}
            self._broadcast_event("open", now, _event_level(info.get("level"),
                                                             diag.get("level_name")),
                                  code, info.get("message") or diag.get("message") or "")
        for code in sorted(self._ros_open_events - cur):
            info = verdicts.get(code) or {}
            self._broadcast_event("close", now, _event_level(info.get("level"),
                                                             diag.get("level_name")),
                                  code, info.get("message") or "")
        self._ros_open_events = cur

    def _broadcast_event(self, action: str, t: float, level: str,
                         code: str, message: str) -> None:
        try:
            self.broadcaster.publish({
                "type": "event", "t": t, "action": action, "replay": False,
                "event": {"t": t, "level": level, "code": code, "message": message},
            })
        except Exception:
            _logger.exception("event broadcast failed")

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
            except Exception:
                # A transient filesystem error must not kill the hourly loop.
                _logger.exception("cleanup failed")
            await asyncio.sleep(_CLEANUP_INTERVAL_S)

    async def shutdown(self) -> None:
        if self._web_server is not None:
            self._web_server.should_exit = True
        for w in (self._corr_log, self._obs_log, self._sol_log, self._can_log):
            if w is not None:
                w.close()
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


def main() -> None:
    if not logging.getLogger().handlers:
        logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(sys.argv[1] if len(sys.argv) > 1 else "config.yaml")
    app = build_app(cfg)
    try:
        asyncio.run(app.run_forever())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
