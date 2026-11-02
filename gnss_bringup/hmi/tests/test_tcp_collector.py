import asyncio

from gnss_hmi.collectors.tcp import TcpCollector


async def _serve_once(port_box: list, payload: bytes, times: int = 2):
    """Server that sends payload then closes, for `times` client connections."""
    remaining = [times]

    async def handler(reader, writer):
        writer.write(payload)
        await writer.drain()
        writer.close()
        remaining[0] -= 1

    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    port_box.append(server.sockets[0].getsockname()[1])
    return server, remaining


async def test_receives_data_and_reconnects():
    port_box: list[int] = []
    server, remaining = await _serve_once(port_box, b"rtcm-bytes")
    got: list[bytes] = []
    events: list[tuple[str, str]] = []
    c = TcpCollector("corr", "127.0.0.1", port_box[0],
                     on_data=lambda d, t: got.append(d),
                     on_event=lambda n, s, det: events.append((n, s)),
                     initial_backoff=0.01, max_backoff=0.05)
    task = asyncio.create_task(c.run())
    for _ in range(200):
        if remaining[0] <= 0 and len(got) >= 2:
            break
        await asyncio.sleep(0.02)
    task.cancel()
    server.close()
    assert b"rtcm-bytes" in got
    assert ("corr", "connected") in events
    assert ("corr", "disconnected") in events
    assert events.count(("corr", "connected")) >= 2  # reconnected after close


async def test_listen_mode_accepts_peer():
    got: list[bytes] = []
    c = TcpCollector("sol", "127.0.0.1", 0, on_data=lambda d, t: got.append(d),
                     on_event=lambda *a: None, listen=True)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)
    assert c.bound_port is not None
    _, writer = await asyncio.open_connection("127.0.0.1", c.bound_port)
    writer.write(b"$GPCHC,...\r\n")
    await writer.drain()
    await asyncio.sleep(0.05)
    writer.close()
    task.cancel()
    assert got == [b"$GPCHC,...\r\n"]


async def test_idle_timeout_treated_as_disconnect():
    """A peer that stops sending (routine on flaky 5G) must not hang the pump
    forever -- it should be treated as a disconnect within idle_timeout."""
    async def handler(reader, writer):
        writer.write(b"hi")
        await writer.drain()
        await asyncio.sleep(10)  # never sends again, never closes

    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]
    got: list[bytes] = []
    events: list[tuple[str, str]] = []
    c = TcpCollector("corr", "127.0.0.1", port,
                     on_data=lambda d, t: got.append(d),
                     on_event=lambda n, s, det: events.append((n, s)),
                     initial_backoff=0.01, max_backoff=0.05,
                     idle_timeout=0.05)
    task = asyncio.create_task(c.run())
    for _ in range(200):
        if ("corr", "disconnected") in events:
            break
        await asyncio.sleep(0.02)
    task.cancel()
    server.close()
    assert b"hi" in got
    assert ("corr", "disconnected") in events


async def test_disconnect_event_only_on_transition():
    """A dead route must not spam a "disconnected" event on every failed
    reconnect attempt -- only on the transition into the disconnected state."""
    port_box: list[int] = []
    server, _remaining = await _serve_once(port_box, b"x", times=1)
    port = port_box[0]
    events: list[tuple[str, str]] = []
    c = TcpCollector("corr", "127.0.0.1", port,
                     on_data=lambda d, t: None,
                     on_event=lambda n, s, det: events.append((n, s)),
                     initial_backoff=0.01, max_backoff=0.01)
    task = asyncio.create_task(c.run())
    for _ in range(200):
        if ("corr", "disconnected") in events:
            break
        await asyncio.sleep(0.01)
    assert ("corr", "disconnected") in events  # the one legitimate transition
    server.close()
    await server.wait_closed()
    events.clear()
    # Now every reconnect attempt fails outright (connection refused); since
    # we never leave the disconnected state, no further events should fire.
    await asyncio.sleep(0.3)
    task.cancel()
    disconnect_count = events.count(("corr", "disconnected"))
    assert disconnect_count == 0, f"expected no repeat disconnect events on dead route, got {disconnect_count}"


async def test_listen_mode_closes_on_cancel():
    """Verify that cancelling listen mode closes accepted connections."""
    c = TcpCollector("sol", "127.0.0.1", 0, on_data=lambda d, t: None,
                     on_event=lambda *a: None, listen=True)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)
    assert c.bound_port is not None
    reader, writer = await asyncio.open_connection("127.0.0.1", c.bound_port)
    await asyncio.sleep(0.05)
    # Record the set of active writers before cancel
    active_before = len(c._active_writers)
    assert active_before > 0, "Should have active writers"
    # Cancel the collector
    task.cancel()
    # Give the event loop several chances to process the cancellation
    # This will eventually cause _run_server to raise CancelledError,
    # which triggers the except block that closes all writers
    closed_at_iteration = None
    for i in range(100):
        await asyncio.sleep(0.01)
        if len(c._active_writers) == 0:
            closed_at_iteration = i
            break
    # After writers are closed, the client should get EOF when reading
    assert closed_at_iteration is not None, "Writers were never closed after cancel"
    data = await asyncio.wait_for(reader.read(1024), timeout=0.5)
    assert data == b"", f"Expected EOF but got {repr(data)}"


async def test_listen_mode_disconnect_event_only_on_transition():
    """Verify that N sequential connect/close cycles produce exactly N 'connected'
    and N 'disconnected' events in per-connection semantics."""
    events = []
    c = TcpCollector("sol", "127.0.0.1", 0, on_data=lambda d, t: None,
                     on_event=lambda n, s, det: events.append(s), listen=True)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)
    # 3 sequential connect/close cycles
    for cycle in range(3):
        _, w = await asyncio.open_connection("127.0.0.1", c.bound_port)
        await asyncio.sleep(0.02)  # Let handler process "connected" event
        w.close()
        await asyncio.sleep(0.05)  # Let handler process "disconnected" event
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    # Exactly N connected and N disconnected events for N cycles
    assert events.count("connected") == 3, f"Expected 3 'connected', got {events.count('connected')}"
    assert events.count("disconnected") == 3, f"Expected 3 'disconnected', got {events.count('disconnected')}"


async def test_listen_mode_concurrent_peers_per_connection_events():
    """Verify per-connection event semantics with concurrent peers.

    Scenario: open two concurrent connections, close peer 1 → poll for exactly
    one 'disconnected'; peer 2 still connected. Close peer 2 → poll for second
    'disconnected'. Assert final counts are 2/2 and first close produced exactly
    one 'disconnected' while peer 2 was still connected.
    """
    events: list[str] = []
    c = TcpCollector("sol", "127.0.0.1", 0, on_data=lambda d, t: None,
                     on_event=lambda n, s, det: events.append(s), listen=True)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)
    assert c.bound_port is not None

    # Open peer 1
    r1, w1 = await asyncio.open_connection("127.0.0.1", c.bound_port)
    await asyncio.sleep(0.05)  # Let handler process "connected"

    # Open peer 2
    r2, w2 = await asyncio.open_connection("127.0.0.1", c.bound_port)
    await asyncio.sleep(0.05)  # Let handler process second "connected"

    # Verify both connected events fired
    assert events.count("connected") == 2, f"Expected 2 'connected' after both peers open, got {events.count('connected')}"

    # Close peer 1 and poll until exactly one "disconnected" event
    w1.close()
    for _ in range(200):
        if events.count("disconnected") == 1:
            break
        await asyncio.sleep(0.01)
    assert events.count("disconnected") == 1, \
        f"Expected exactly 1 'disconnected' after peer 1 close, got {events.count('disconnected')}"

    # Verify peer 2 still has open connection (no second disconnected yet)
    data = b"test"
    w2.write(data)
    await w2.drain()
    # Peer 2 should still be connected, so the send should succeed without exception
    assert events.count("disconnected") == 1, \
        f"Peer 2 should still be connected, but got {events.count('disconnected')} disconnected events"

    # Close peer 2 and poll until second "disconnected" event
    w2.close()
    for _ in range(200):
        if events.count("disconnected") == 2:
            break
        await asyncio.sleep(0.01)
    assert events.count("disconnected") == 2, \
        f"Expected exactly 2 'disconnected' after peer 2 close, got {events.count('disconnected')}"

    # Final counts should be 2/2
    assert events.count("connected") == 2, \
        f"Final: expected 2 'connected', got {events.count('connected')}"
    assert events.count("disconnected") == 2, \
        f"Final: expected 2 'disconnected', got {events.count('disconnected')}"

    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
