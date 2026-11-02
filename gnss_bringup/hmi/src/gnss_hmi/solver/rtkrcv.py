"""Generate rtkrcv.conf and supervise the rtkrcv subprocess.

Config keys target RTKLIB demo5; verifying exact key names against the real
binary is an integration step (docs/integration-rtkrcv.md), not a unit test.
"""
from __future__ import annotations

import asyncio
import logging
import os
import signal
import time
from pathlib import Path
from typing import Callable

_logger = logging.getLogger(__name__)

# Crash-loop backoff: a process that dies almost immediately (bad binary,
# misconfigured conf) would otherwise restart at a fixed cadence forever,
# generating tens of thousands of connected/disconnected rows a day. If a
# life is shorter than this, double the restart delay (capped); a life at
# least this long resets the delay back to the configured base.
_CRASH_LOOP_LIFE_S = 30.0
_MAX_RESTART_DELAY_S = 60.0


def _signal_group(proc, sig):
    """Signal a process group to ensure child and grandchild termination."""
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        pass

_CONF_TEMPLATE = """\
inpstr1-type =tcpcli
inpstr1-path =127.0.0.1:{obs_port}
inpstr1-format =rtcm3
inpstr2-type =tcpcli
inpstr2-path =127.0.0.1:{corr_port}
inpstr2-format =rtcm3
outstr1-type =tcpsvr
outstr1-path =:{sol_port}
outstr1-format =llh
pos1-posmode =kinematic
pos1-navsys =63
pos1-elmask =10
pos2-armode =continuous
ant2-postype =rtcm
out-solformat =llh
out-outhead =off
out-timesys =gpst
misc-svrcycle =10
"""


class RtkrcvManager:
    def __init__(self, binary: str, run_dir: Path, corr_port: int, obs_port: int,
                 sol_port: int, extra_args: tuple[str, ...] = (),
                 restart_delay: float = 5.0,
                 on_event: Callable[[str, str, str], None] | None = None) -> None:
        self._binary = binary
        # Resolve to absolute: rtkrcv is spawned with cwd=run_dir, so a
        # relative conf path (from a relative data_root) would be resolved
        # against run_dir a second time — double-nested, not found, and the
        # solver silently runs with no config. Absolute is cwd-independent.
        self._run_dir = Path(run_dir).resolve()
        self._corr_port = corr_port
        self._obs_port = obs_port
        self._sol_port = sol_port
        self._extra = extra_args
        self._delay = restart_delay
        self._on_event = on_event
        self.current_delay = restart_delay
        self._last_state: str | None = None

    def write_conf(self) -> Path:
        self._run_dir.mkdir(parents=True, exist_ok=True)
        conf = self._run_dir / "rtkrcv.conf"
        conf.write_text(_CONF_TEMPLATE.format(
            obs_port=self._obs_port, corr_port=self._corr_port,
            sol_port=self._sol_port))
        return conf

    async def run(self) -> None:
        conf = self.write_conf()
        env = dict(os.environ, RTKRCV_SOL_PORT=str(self._sol_port))
        while True:
            proc = None
            started = time.monotonic()
            try:
                # extra_args precede the fixed flags: lets tests spawn
                # "python fake_rtkrcv.py ..." (Task 13); empty for the real binary.
                # -r 2 writes a rtkrcv_<time>.stat file (per-satellite $SAT lines
                # with az/el/snr) into cwd=run_dir, which the app tails to feed
                # the skyplot.
                proc = await asyncio.create_subprocess_exec(
                    self._binary, *self._extra, "-s", "-nc", "-r", "2", "-o", str(conf),
                    cwd=self._run_dir, env=env,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL,
                    start_new_session=True)
                self._emit("connected", f"pid {proc.pid}")
                rc = await proc.wait()
                self._emit("disconnected", f"exit code {rc}")
                self._update_delay(time.monotonic() - started)
            except asyncio.CancelledError:
                if proc is not None and proc.returncode is None:
                    _signal_group(proc, signal.SIGTERM)
                    try:
                        await asyncio.wait_for(proc.wait(), 5.0)
                    except asyncio.TimeoutError:
                        _signal_group(proc, signal.SIGKILL)
                        await proc.wait()
                raise
            except Exception:
                _logger.exception("rtkrcv spawn failed")
                self._emit("disconnected", "spawn failed")
                self._update_delay(time.monotonic() - started)
            await asyncio.sleep(self.current_delay)

    def _emit(self, state: str, detail: str) -> None:
        # Dedup on transition only (like TcpCollector): a process that keeps
        # failing to spawn would otherwise emit "disconnected" every restart
        # cycle forever with no intervening "connected".
        if self._last_state == state:
            return
        self._last_state = state
        if self._on_event:
            self._on_event("rtkrcv", state, detail)

    def _update_delay(self, life_s: float) -> None:
        if life_s < _CRASH_LOOP_LIFE_S:
            self.current_delay = min(self.current_delay * 2, _MAX_RESTART_DELAY_S)
        else:
            self.current_delay = self._delay
