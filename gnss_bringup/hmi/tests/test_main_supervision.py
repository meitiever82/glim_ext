"""Failure isolation: a crashing callback or collector must not take down
the whole daemon -- "collection never stops" is the spec's core principle.
"""
import asyncio
import textwrap

import gnss_hmi.main as main_mod
from gnss_hmi.config import load_config
from gnss_hmi.main import build_app


def _build_app(tmp_path, corr_port=0, corr_listen=True):
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: {corr_port}, listen: {str(corr_listen).lower()}}}
        raw_obs: {{host: 127.0.0.1, port: 0, listen: true}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:sup_{tmp_path.name}
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        """))
    return build_app(load_config(cfg_file))


async def test_on_event_guarded_against_storage_failure(tmp_path):
    app = _build_app(tmp_path)

    def boom(*a, **k):
        raise RuntimeError("db is locked")

    app.events.record = boom
    # Must not raise -- a storage failure here must never propagate into a
    # collector's callback chain.
    app._on_event("corrections_link", "connected", "")
    app._bus.shutdown()
    app.events.close()


async def test_on_corr_broadcast_survives_append_failure(tmp_path):
    app = _build_app(tmp_path)
    broadcasted: list[bytes] = []
    app.corr_reserver.broadcast = lambda data: broadcasted.append(data)

    def boom_feed(data):
        raise RuntimeError("parse blew up")

    app._corr_framer.feed = boom_feed
    # The rawlog-append step raises, but broadcast is a separate guarded
    # step (spec Sec3.3): a failed append must not skip the broadcast.
    app._on_corr(b"whatever-bytes", 0.0)
    assert broadcasted == [b"whatever-bytes"]
    app._bus.shutdown()
    app.events.close()


async def test_on_can_guarded_against_log_failure(tmp_path):
    app = _build_app(tmp_path)

    def boom(*a, **k):
        raise OSError("disk full")

    app._can_log.append = boom
    # Must not raise.
    app._on_can(0x123, b"\x01\x02", 0.0)
    app._bus.shutdown()
    app.events.close()


async def test_event_store_failure_does_not_kill_collectors(tmp_path):
    """End-to-end: even if every EventStore.record call raises, run_forever
    (and thus every other route) must keep running."""
    async def handler(reader, writer):
        writer.write(b"hello-corr")
        await writer.drain()
        await asyncio.sleep(10)

    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]
    app = _build_app(tmp_path, corr_port=port, corr_listen=False)

    def boom(*a, **k):
        raise RuntimeError("db locked")

    app.events.record = boom

    task = asyncio.create_task(app.run_forever())
    await asyncio.sleep(0.3)
    assert not task.done(), "run_forever must not crash when EventStore.record raises"
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    await app.shutdown()
    server.close()
    await server.wait_closed()


async def test_supervisor_restarts_crashed_collector(monkeypatch, tmp_path):
    monkeypatch.setattr(main_mod, "_SUPERVISE_RESTART_S", 0.01)
    app = _build_app(tmp_path)
    attempts: list[int] = []

    async def flaky():
        attempts.append(1)
        if len(attempts) == 1:
            raise RuntimeError("boom")
        await asyncio.sleep(10)

    task = asyncio.create_task(app._supervise("flaky", flaky))
    for _ in range(200):
        if len(attempts) >= 2:
            break
        await asyncio.sleep(0.01)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert len(attempts) >= 2, "supervisor should have restarted the crashed coroutine"
    states = [(e.etype, e.state) for e in app.events.query()]
    assert ("flaky", "crashed") in states
    app._bus.shutdown()
    app.events.close()


async def test_supervisor_sleeps_after_clean_return(monkeypatch, tmp_path):
    """I6: _supervise only slept on exception -- a coroutine that returns
    cleanly (e.g. uvicorn's serve() after should_exit) respun instantly,
    hot-looping the supervisor. A clean return must also incur the restart
    delay."""
    monkeypatch.setattr(main_mod, "_SUPERVISE_RESTART_S", 0.05)
    app = _build_app(tmp_path)
    attempts: list[int] = []

    async def clean_return():
        attempts.append(1)
        return

    task = asyncio.create_task(app._supervise("clean", clean_return))
    # With a 0.05s restart delay: call1 at t=0, call2 at t~0.05, then a sleep
    # still pending at t=0.08 -- an unthrottled hot loop would instead have
    # racked up hundreds of calls in this window.
    await asyncio.sleep(0.08)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert len(attempts) <= 2, "supervisor must sleep between clean-return respins, not hot-loop"
    app._bus.shutdown()
    app.events.close()


async def test_socketcan_branch_wires_bus_factory(monkeypatch, tmp_path):
    """T11 built CanCollector's reopen watchdog, but App only ever passed a
    bus_factory=None -- dead code in production. The real (non-virtual)
    socketcan branch must supply a bus_factory that reopens the same
    channel."""
    calls: list[tuple[str, str]] = []

    class FakeBus:
        def shutdown(self):
            pass

    def fake_can_bus(interface, channel):
        calls.append((interface, channel))
        return FakeBus()

    monkeypatch.setattr(main_mod.can, "Bus", fake_can_bus)
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 0, listen: true}}
        raw_obs: {{host: 127.0.0.1, port: 0, listen: true}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: can0
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        """))
    app = build_app(load_config(cfg_file))
    assert calls == [("socketcan", "can0")]           # __init__'s own bus
    factory = app._can_collector._bus_factory
    assert factory is not None
    factory()
    assert calls == [("socketcan", "can0"), ("socketcan", "can0")]


async def test_supervisor_propagates_cancellation(monkeypatch, tmp_path):
    monkeypatch.setattr(main_mod, "_SUPERVISE_RESTART_S", 5.0)
    app = _build_app(tmp_path)

    async def hang():
        await asyncio.sleep(10)

    task = asyncio.create_task(app._supervise("hang", hang))
    await asyncio.sleep(0.05)
    task.cancel()
    try:
        await asyncio.wait_for(task, timeout=1.0)
        assert False, "expected CancelledError"
    except asyncio.CancelledError:
        pass
    app._bus.shutdown()
    app.events.close()
