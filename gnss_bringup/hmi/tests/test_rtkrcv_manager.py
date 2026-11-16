import asyncio
import os
import stat

import pytest

from gnss_hmi.solver.rtkrcv import RtkrcvManager


def _fake_binary(tmp_path, body: str) -> str:
    p = tmp_path / "fake_rtkrcv"
    p.write_text("#!/bin/sh\n" + body)
    p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return str(p)


def test_write_conf(tmp_path):
    m = RtkrcvManager("/bin/true", tmp_path, corr_port=15010, obs_port=15011,
                      sol_port=15020)
    conf = m.write_conf()
    text = conf.read_text()
    assert "inpstr1-path =127.0.0.1:15011" in text      # rover = raw obs
    assert "inpstr2-path =127.0.0.1:15010" in text      # base = corrections
    assert "outstr1-path =:15020" in text
    assert "pos1-posmode =kinematic" in text
    assert "ant2-postype =rtcm" in text                 # base pos from RTCM 1005/1006


async def test_restarts_after_exit_and_terminates_on_cancel(tmp_path):
    marker = tmp_path / "runs"
    binary = _fake_binary(
        tmp_path, f'echo run >> "{marker}"\nsleep 0.1\n')
    events = []
    m = RtkrcvManager(binary, tmp_path, 15010, 15011, 15020,
                      restart_delay=0.05,
                      on_event=lambda n, s, d: events.append((n, s)))
    task = asyncio.create_task(m.run())
    await asyncio.sleep(0.6)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert marker.read_text().count("run") >= 2          # restarted at least once
    assert ("rtkrcv", "disconnected") in events


async def test_sol_port_in_env(tmp_path):
    out = tmp_path / "env"
    binary = _fake_binary(tmp_path, f'echo "$RTKRCV_SOL_PORT" > "{out}"\nsleep 5\n')
    m = RtkrcvManager(binary, tmp_path, 15010, 15011, 15021, restart_delay=9)
    task = asyncio.create_task(m.run())
    await asyncio.sleep(0.3)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert out.read_text().strip() == "15021"


async def test_crash_loop_backoff_and_dedup(tmp_path):
    """A binary that never manages to spawn (bad path) must not flood
    'disconnected' rows every restart_delay forever, and the restart delay
    must back off exponentially instead of hammering the crash loop."""
    events: list[tuple[str, str]] = []
    m = RtkrcvManager(str(tmp_path / "does-not-exist"), tmp_path, 15010, 15011,
                      15020, restart_delay=0.02,
                      on_event=lambda n, s, d: events.append((n, s)))
    task = asyncio.create_task(m.run())
    # Let several restart attempts happen; each spawn fails immediately
    # (FileNotFoundError), so this is a fast crash loop.
    for _ in range(200):
        if m.current_delay > 0.02:
            break
        await asyncio.sleep(0.01)
    await asyncio.sleep(0.1)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    # Delay grew from the base (crash-loop backoff), capped at 60s.
    assert m.current_delay > 0.02
    assert m.current_delay <= 60.0
    # No "connected" event ever fired (spawn always fails) -- dedup must
    # collapse the repeated "disconnected" transitions into a single row.
    assert events.count(("rtkrcv", "disconnected")) == 1
    assert ("rtkrcv", "connected") not in events


async def test_no_orphaned_grandchildren_on_cancel(tmp_path):
    """Verify that forking children (grandchildren) are killed on cancellation."""
    pidfile = tmp_path / "grandchild.pid"
    # Spawn a grandchild: 'sleep 30 & echo $! > pidfile; wait'
    binary = _fake_binary(
        tmp_path,
        f'sleep 30 & echo $! > "{pidfile}"\nwait\n'
    )
    m = RtkrcvManager(binary, tmp_path, 15010, 15011, 15020, restart_delay=9)
    task = asyncio.create_task(m.run())
    # Wait for grandchild to be spawned and pidfile written
    await asyncio.sleep(0.3)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    # Brief delay to ensure process reaping completes
    await asyncio.sleep(0.2)
    # Verify grandchild PID is no longer alive
    if pidfile.exists():
        grandchild_pid = int(pidfile.read_text().strip())
        with pytest.raises(ProcessLookupError):
            os.kill(grandchild_pid, 0)  # signal 0 checks if process exists


async def test_conf_loadable_with_relative_run_dir(tmp_path, monkeypatch):
    """Regression: a relative data_root made the manager pass a relative -o conf
    path while spawning rtkrcv with cwd=run_dir, so rtkrcv resolved the conf
    against run_dir (double-nesting) and silently ran with no config."""
    monkeypatch.chdir(tmp_path)
    marker = tmp_path / "conf_check"          # absolute, immune to cwd games
    # fake rtkrcv: locate the token after -o and report whether that file is
    # findable from the process's own cwd (which the manager sets to run_dir).
    body = (
        'conf=""\n'
        'while [ $# -gt 0 ]; do\n'
        '  if [ "$1" = "-o" ]; then shift; conf="$1"; fi\n'
        '  shift\n'
        'done\n'
        f'if [ -f "$conf" ]; then echo found > "{marker}"; '
        f'else echo "missing:$conf" > "{marker}"; fi\n'
        'sleep 5\n'
    )
    binary = _fake_binary(tmp_path, body)
    m = RtkrcvManager(binary, "rundir", 15010, 15011, 15020, restart_delay=9)
    task = asyncio.create_task(m.run())
    await asyncio.sleep(0.4)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert marker.read_text().strip() == "found"
