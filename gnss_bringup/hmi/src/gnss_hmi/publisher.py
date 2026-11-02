"""Best-effort UDP JSON Lines publisher (spec §7, GLIM phase-2 interface)."""
from __future__ import annotations

import asyncio
import json

from gnss_hmi.diagnosis.rules import Verdict
from gnss_hmi.parsers.rtksol import RtkSolution


class UdpPublisher:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            asyncio.DatagramProtocol, remote_addr=(self._host, self._port))

    def _send(self, obj: dict) -> None:
        if self._transport is None or self._transport.is_closing():
            return
        try:
            self._transport.sendto((json.dumps(obj, ensure_ascii=False) + "\n").encode())
        except OSError:
            pass

    def publish_fix(self, sol: RtkSolution, heading: float | None = None,
                    host_time: float | None = None) -> None:
        self._send({"type": "gnss_fix", "ver": 1, "gps_time": sol.t,
                    "host_time": host_time, "lat": sol.lat, "lon": sol.lon,
                    "alt": sol.alt, "q": sol.q, "sigma_e": sol.sde, "sigma_n": sol.sdn,
                    "sigma_u": sol.sdu, "heading": heading, "source": "rtkrcv"})

    def publish_event(self, kind: str, verdict: Verdict, t: float) -> None:
        self._send({"type": "gnss_event", "ver": 1, "gps_time": t,
                    "host_time": t, "event": verdict.code, "state": kind,
                    "detail": verdict.message})

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None
