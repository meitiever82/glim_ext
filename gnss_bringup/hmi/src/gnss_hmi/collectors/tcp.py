"""TCP stream collector: connect-or-listen, deliver raw chunks, reconnect forever.

The collector performs no parsing — its only jobs are delivering bytes with a
host timestamp and reporting link state transitions.
"""
from __future__ import annotations

import asyncio
import time
from typing import Callable

OnData = Callable[[bytes, float], None]
OnEvent = Callable[[str, str, str], None]


class TcpCollector:
    def __init__(self, name: str, host: str, port: int,
                 on_data: OnData, on_event: OnEvent, listen: bool = False,
                 initial_backoff: float = 1.0, max_backoff: float = 30.0,
                 idle_timeout: float = 30.0) -> None:
        self._name = name
        self._host = host
        self._port = port
        self._on_data = on_data
        self._on_event = on_event
        self._listen = listen
        self._initial = initial_backoff
        self._max = max_backoff
        self._idle_timeout = idle_timeout
        self.bound_port: int | None = None
        self._active_writers: set[asyncio.StreamWriter] = set()
        self._last_state: str | None = None

    @property
    def name(self) -> str:
        return self._name

    async def run(self) -> None:
        if self._listen:
            await self._run_server()
        else:
            await self._run_client()

    async def _pump(self, reader: asyncio.StreamReader) -> None:
        self._on_event(self._name, "connected", "")
        self._last_state = "connected"
        while True:
            try:
                data = await asyncio.wait_for(reader.read(4096), timeout=self._idle_timeout)
            except asyncio.TimeoutError:
                # Peer vanished without RST (routine on flaky links) -- treat
                # the silence as a disconnect so the reconnect path runs.
                break
            if not data:
                break
            self._on_data(data, time.time())

    async def _run_client(self) -> None:
        backoff = self._initial
        while True:
            writer = None
            try:
                reader, writer = await asyncio.open_connection(self._host, self._port)
                backoff = self._initial
                await self._pump(reader)
            except OSError:
                pass
            finally:
                if writer is not None:
                    writer.close()
            if self._last_state != "disconnected":
                self._on_event(self._name, "disconnected", f"retry in {backoff:.0f}s")
                self._last_state = "disconnected"
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, self._max)

    async def _run_server(self) -> None:
        async def handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
            self._active_writers.add(writer)
            was_connected = False
            try:
                self._on_event(self._name, "connected", "")
                was_connected = True
                try:
                    while True:
                        try:
                            data = await asyncio.wait_for(reader.read(4096), timeout=self._idle_timeout)
                        except asyncio.TimeoutError:
                            break
                        if not data:
                            break
                        self._on_data(data, time.time())
                except OSError:
                    pass
            finally:
                if was_connected:
                    self._on_event(self._name, "disconnected", "peer closed")
                writer.close()
                self._active_writers.discard(writer)

        server = await asyncio.start_server(handler, self._host, self._port)
        self.bound_port = server.sockets[0].getsockname()[1]
        try:
            async with server:
                await server.serve_forever()
        except asyncio.CancelledError:
            # Close all accepted connections before exiting
            for w in list(self._active_writers):
                w.close()
            raise
