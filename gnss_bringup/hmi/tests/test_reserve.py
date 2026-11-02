import asyncio
from unittest import mock

from gnss_hmi.collectors.reserve import LocalReserver


async def test_broadcast_to_multiple_clients():
    r = LocalReserver()
    await r.start(0)
    async def client():
        reader, writer = await asyncio.open_connection("127.0.0.1", r.bound_port)
        data = await reader.readexactly(5)
        writer.close()
        return data
    t1, t2 = asyncio.create_task(client()), asyncio.create_task(client())
    await asyncio.sleep(0.05)
    r.broadcast(b"hello")
    assert await t1 == b"hello" and await t2 == b"hello"
    await r.stop()


async def test_broadcast_with_no_clients_is_noop():
    r = LocalReserver()
    await r.start(0)
    r.broadcast(b"x")  # must not raise
    await r.stop()


async def test_client_reads_are_bounded_not_unbounded():
    """_on_client must read in bounded chunks, not `read()` with no limit
    (which buffers everything a client sends until EOF)."""
    calls: list[int] = []
    orig_read = asyncio.StreamReader.read

    async def spy_read(self, n=-1):
        calls.append(n)
        return await orig_read(self, n)

    with mock.patch.object(asyncio.StreamReader, "read", spy_read):
        r = LocalReserver()
        await r.start(0)
        reader, writer = await asyncio.open_connection("127.0.0.1", r.bound_port)
        await asyncio.sleep(0.05)
        writer.write(b"junk-from-client")
        await writer.drain()
        await asyncio.sleep(0.05)
        writer.close()
        await asyncio.sleep(0.05)
        assert len(r._writers) == 0, "writer should be discarded once client disconnects"
        await r.stop()
    assert calls, "expected _on_client to read from the stream"
    assert all(n != -1 for n in calls), f"reads must be bounded, got unbounded calls: {calls}"


async def test_client_data_is_discarded_but_broadcast_still_works():
    r = LocalReserver()
    await r.start(0)
    reader, writer = await asyncio.open_connection("127.0.0.1", r.bound_port)
    await asyncio.sleep(0.05)
    writer.write(b"x" * (256 * 1024))  # larger than a single bounded read
    await writer.drain()
    await asyncio.sleep(0.1)
    assert len(r._writers) == 1, "connection should still be tracked while open"
    r.broadcast(b"still-works")
    got = await reader.readexactly(len(b"still-works"))
    assert got == b"still-works"
    writer.close()
    await asyncio.sleep(0.05)
    assert len(r._writers) == 0, "writer should be discarded once client disconnects"
    await r.stop()


async def test_broadcast_force_closes_stalled_clients():
    r = LocalReserver()
    await r.start(0)

    async def stalled_client():
        reader, writer = await asyncio.open_connection("127.0.0.1", r.bound_port)
        # Don't read — simulate a stalled client
        await asyncio.sleep(10)

    task = asyncio.create_task(stalled_client())
    await asyncio.sleep(0.05)  # Let client connect

    # Monkeypatch the transport's get_write_buffer_size to simulate a full buffer
    writer = list(r._writers)[0]
    writer.transport.get_write_buffer_size = lambda: 10 * 1024 * 1024

    # Broadcast should force-close the stalled client
    r.broadcast(b"x")

    # Assert the writer was removed and is closing
    assert writer not in r._writers
    assert writer.is_closing()

    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    await r.stop()
