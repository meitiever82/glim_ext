"""SocketCAN (or any python-can bus) collector."""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Callable

import can

OnFrame = Callable[[int, bytes, float], None]
OnEvent = Callable[[str, str, str], None]

_logger = logging.getLogger(__name__)


class CanCollector:
    def __init__(self, bus: can.BusABC, on_frame: OnFrame,
                 on_event: OnEvent | None = None, data_timeout: float = 2.0,
                 bus_factory: Callable[[], can.BusABC] | None = None, reopen_after: int = 5) -> None:
        self._bus = bus
        self._on_frame = on_frame
        self._on_event = on_event
        self._data_timeout = data_timeout
        self._bus_factory = bus_factory
        self._reopen_after = reopen_after
        self._seen_any = False
        self._was_down = False
        self._reopen_failed_emitted = False

    def _emit(self, name: str, state: str, detail: str) -> None:
        if self._on_event is not None:
            self._on_event(name, state, detail)

    async def run(self) -> None:
        bus = self._bus
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
        timeouts = 0
        try:
            while True:
                try:
                    msg = await asyncio.wait_for(reader.get_message(),
                                                 timeout=self._data_timeout)
                except asyncio.TimeoutError:
                    timeouts += 1
                    if timeouts == 1 and self._on_event:
                        self._was_down = True
                        self._on_event("can_link", "disconnected",
                                       f"no frames for {self._data_timeout:.0f}s")
                    if (self._bus_factory is not None
                            and timeouts >= self._reopen_after):
                        try:
                            notifier.stop()
                            bus.shutdown()
                            bus = self._bus_factory()
                            reader = can.AsyncBufferedReader()
                            notifier = can.Notifier(bus, [reader],
                                                    loop=asyncio.get_running_loop())
                            if self._on_event:
                                self._on_event("can_link", "reopened", "bus reopened")
                            timeouts = 0
                            self._reopen_failed_emitted = False
                        except Exception:
                            # Reopen itself failed (e.g. interface still down).
                            # Don't let this kill the collector task -- log it,
                            # emit one event (not one per retry), and keep the
                            # old (now-dead) bus/notifier around so the loop
                            # keeps ticking on timeouts and retries reopening
                            # next time the threshold is hit.
                            _logger.exception("can bus reopen failed")
                            if self._on_event and not self._reopen_failed_emitted:
                                self._on_event("can_link", "reopen_failed",
                                               "bus reopen failed")
                                self._reopen_failed_emitted = True
                    continue
                if self._was_down or not self._seen_any:
                    if self._on_event:
                        self._on_event("can_link", "connected", "")
                    self._seen_any = True
                    self._was_down = False
                    timeouts = 0
                self._on_frame(msg.arbitration_id, bytes(msg.data),
                               msg.timestamp or time.time())
        finally:
            notifier.stop()
