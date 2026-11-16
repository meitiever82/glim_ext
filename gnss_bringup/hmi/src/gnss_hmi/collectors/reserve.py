"""Localhost TCP fan-out: re-serve a collected stream to local consumers (rtkrcv)."""
from __future__ import annotations

import asyncio

_MAX_WRITE_BUFFER = 4 * 1024 * 1024  # force-close clients that stop reading


class LocalReserver:
    def __init__(self) -> None:
        self._writers: set[asyncio.StreamWriter] = set()
        self._server: asyncio.Server | None = None
        self.bound_port: int | None = None

    async def start(self, port: int, host: str = "127.0.0.1") -> None:
        self._server = await asyncio.start_server(self._on_client, host, port)
        self.bound_port = self._server.sockets[0].getsockname()[1]

    async def _on_client(self, reader: asyncio.StreamReader,
                         writer: asyncio.StreamWriter) -> None:
        self._writers.add(writer)
        try:
            # Clients aren't expected to send anything -- discard in bounded
            # chunks (rather than a single unbounded read()) until EOF so a
            # misbehaving client can't buffer unboundedly in memory.
            while await reader.read(4096):
                pass
        finally:
            self._writers.discard(writer)
            writer.close()

    def broadcast(self, data: bytes) -> None:
        for w in list(self._writers):
            if w.is_closing():
                self._writers.discard(w)
                continue
            if w.transport.get_write_buffer_size() > _MAX_WRITE_BUFFER:
                self._writers.discard(w)
                w.close()
                continue
            w.write(data)

    async def stop(self) -> None:
        for w in list(self._writers):
            w.close()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
