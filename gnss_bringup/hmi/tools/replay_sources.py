#!/usr/bin/env python3
# tools/replay_sources.py
"""Replay recorded streams as live sources for end-to-end testing without a vehicle.

Usage:
  python tools/replay_sources.py --candump can.log --can-channel virtual:e2e \
      --rtcm corr.rtcm3 --rtcm-port 6001 --speed 10

Serves each --rtcm/--text file on its TCP port (loops forever), and replays a
candump log onto a python-can channel, all paced by original timestamps / speed.
"""
from __future__ import annotations

import argparse
import asyncio
import re

import can

LINE_RE = re.compile(r"\((?P<t>[\d.]+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)")


async def serve_file(path: str, port: int, chunk: int, interval: float) -> None:
    data = open(path, "rb").read()

    async def handler(reader, writer):
        try:
            while True:
                for off in range(0, len(data), chunk):
                    writer.write(data[off:off + chunk])
                    await writer.drain()
                    await asyncio.sleep(interval)
        except (ConnectionResetError, BrokenPipeError):
            pass

    server = await asyncio.start_server(handler, "0.0.0.0", port)
    print(f"serving {path} on :{port}")
    async with server:
        await server.serve_forever()


async def replay_candump(path: str, channel: str, speed: float) -> None:
    if channel.startswith("virtual:"):
        bus = can.Bus(interface="virtual", channel=channel.split(":", 1)[1])
    else:
        bus = can.Bus(interface="socketcan", channel=channel)
    prev_t = None
    with open(path) as f:
        for line in f:
            m = LINE_RE.match(line)
            if not m:
                continue
            t = float(m["t"])
            if prev_t is not None:
                await asyncio.sleep(max(0.0, (t - prev_t) / speed))
            prev_t = t
            bus.send(can.Message(arbitration_id=int(m["id"], 16),
                                 data=bytes.fromhex(m["data"]), is_extended_id=False))
    bus.shutdown()
    print("candump replay finished")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candump")
    ap.add_argument("--can-channel", default="virtual:e2e")
    ap.add_argument("--rtcm", action="append", default=[],
                    help="FILE:PORT, may repeat (corrections, raw obs)")
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    async def amain():
        tasks = []
        for spec in args.rtcm:
            path, port = spec.rsplit(":", 1)
            tasks.append(serve_file(path, int(port), chunk=512, interval=0.1 / args.speed))
        if args.candump:
            tasks.append(replay_candump(args.candump, args.can_channel, args.speed))
        await asyncio.gather(*tasks)

    asyncio.run(amain())


if __name__ == "__main__":
    main()
