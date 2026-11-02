import asyncio

import can

from gnss_hmi.collectors.can import CanCollector
from gnss_hmi.storage.canlog import CandumpWriter


async def test_collector_receives_virtual_bus_frames():
    with can.Bus(interface="virtual", channel="t0") as tx, \
         can.Bus(interface="virtual", channel="t0") as rx:
        got: list[tuple[int, bytes]] = []
        c = CanCollector(rx, on_frame=lambda i, d, t: got.append((i, d)))
        task = asyncio.create_task(c.run())
        await asyncio.sleep(0.05)
        tx.send(can.Message(arbitration_id=0x320, data=b"\x01\x02", is_extended_id=False))
        for _ in range(100):
            if got:
                break
            await asyncio.sleep(0.01)
        task.cancel()
        assert got == [(0x320, b"\x01\x02")]


async def test_link_watchdog_fires_disconnected_then_connected():
    """No frames for data_timeout -> a "disconnected" can_link event (once
    per outage); the first frame after that -> a "connected" event."""
    with can.Bus(interface="virtual", channel="t2") as tx, \
         can.Bus(interface="virtual", channel="t2") as rx:
        events: list[tuple[str, str, str]] = []
        c = CanCollector(rx, on_frame=lambda i, d, t: None,
                          on_event=lambda n, s, d: events.append((n, s, d)),
                          data_timeout=0.05)
        task = asyncio.create_task(c.run())
        for _ in range(200):
            if ("can_link", "disconnected") in [(n, s) for n, s, _ in events]:
                break
            await asyncio.sleep(0.01)
        states = [(n, s) for n, s, _ in events]
        assert ("can_link", "disconnected") in states
        # No frame yet -> only one disconnected event fired, not a repeat per poll.
        assert states.count(("can_link", "disconnected")) == 1

        tx.send(can.Message(arbitration_id=0x321, data=b"\x00", is_extended_id=False))
        for _ in range(200):
            if ("can_link", "connected") in [(n, s) for n, s, _ in events]:
                break
            await asyncio.sleep(0.01)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        states = [(n, s) for n, s, _ in events]
        assert ("can_link", "connected") in states


async def test_reopen_failure_survives_and_emits_event():
    """If bus_factory raises during a reopen attempt, the collector task
    must not die -- log it, emit one 'reopen_failed' event, and keep the
    supervise-friendly loop going (next timeout retries)."""
    def failing_factory():
        raise OSError("no such device")

    events: list[str] = []
    with can.Bus(interface="virtual", channel="reopen-fail-test") as first:
        c = CanCollector(first, on_frame=lambda *a: None,
                         on_event=lambda n, s, d: events.append(s),
                         data_timeout=0.05, bus_factory=failing_factory,
                         reopen_after=2)
        task = asyncio.create_task(c.run())
        for _ in range(300):
            if "reopen_failed" in events:
                break
            await asyncio.sleep(0.01)
        assert "reopen_failed" in events
        assert not task.done(), "collector task must survive a failed reopen"
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


def test_candump_writer_format(tmp_path):
    w = CandumpWriter(tmp_path, "can0", clock=lambda: 1756699200.0)
    w.append(0x320, bytes.fromhex("44093c1cc30600aa"), t=1756699200.123456)
    w.close()
    day = next(tmp_path.iterdir())
    line = next(day.glob("can0_*.log")).read_text().strip()
    assert line == "(1756699200.123456) can0 320#44093C1CC30600AA"
    # No per-frame JSONL sidecar: candump lines already carry timestamps, so
    # the index would be pure waste at CAN frame rates (~700/s).
    assert list(day.glob("*.idx.jsonl")) == []


async def test_bus_reopened_after_consecutive_timeouts():
    made = []
    def factory():
        bus = can.Bus(interface="virtual", channel="reopen-test")
        made.append(bus)
        return bus
    events = []
    first = factory()
    c = CanCollector(first, on_frame=lambda *a: None,
                     on_event=lambda n, s, d: events.append(s),
                     data_timeout=0.05, bus_factory=factory, reopen_after=2)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.5)                  # several timeouts → at least one reopen
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert "reopened" in events and len(made) >= 2
    for b in made:
        try: b.shutdown()
        except Exception: pass


async def test_can_recovery_event_after_reopen():
    """Verify that after bus reopen, the next frame emits a 'connected' event.

    Scenario: send frame 1 (connected) → timeout to disconnection → wait for
    reopened event → send frame on NEW bus → verify second 'connected' appears
    after the 'reopened' event.
    """
    made = []
    def factory():
        bus = can.Bus(interface="virtual", channel="recovery-test-rx")
        made.append(bus)
        return bus

    events: list[str] = []
    first = factory()
    # Create separate tx bus for sending frames on the same channel
    tx = can.Bus(interface="virtual", channel="recovery-test-rx")

    c = CanCollector(first, on_frame=lambda *a: None,
                     on_event=lambda n, s, d: events.append(s),
                     data_timeout=0.05, bus_factory=factory, reopen_after=2)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)

    # Send first frame on tx bus → should trigger "connected"
    tx.send(can.Message(arbitration_id=0x100, data=b"\x01", is_extended_id=False))
    for _ in range(200):
        if "connected" in events:
            break
        await asyncio.sleep(0.01)
    assert "connected" in events, "First connected event should fire"

    # Wait for timeout → "disconnected" event
    for _ in range(200):
        if "disconnected" in events:
            break
        await asyncio.sleep(0.01)
    assert "disconnected" in events, "Disconnected event should fire after timeout"

    # Wait for reopen event (after 2 consecutive timeouts)
    for _ in range(500):
        if "reopened" in events:
            break
        await asyncio.sleep(0.01)
    assert "reopened" in events, "Reopened event should fire"
    reopen_index = events.index("reopened")

    # Send frame on the same tx channel (will be read by the new bus from factory)
    tx.send(can.Message(arbitration_id=0x200, data=b"\x02", is_extended_id=False))

    # Wait for second "connected" event AFTER "reopened"
    second_connected_index = None
    for _ in range(200):
        connected_indices = [i for i, e in enumerate(events) if e == "connected"]
        if len(connected_indices) >= 2 and connected_indices[-1] > reopen_index:
            second_connected_index = connected_indices[-1]
            break
        await asyncio.sleep(0.01)

    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass

    tx.shutdown()
    for b in made:
        try: b.shutdown()
        except Exception: pass

    assert second_connected_index is not None, \
        f"Second 'connected' should appear after 'reopened'. Events: {events}"
    assert second_connected_index > reopen_index, \
        f"Second 'connected' (index {second_connected_index}) should be after 'reopened' (index {reopen_index})"
