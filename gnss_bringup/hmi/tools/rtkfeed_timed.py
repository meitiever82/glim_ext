#!/usr/bin/env python3
# tools/rtkfeed_timed.py
"""Time-synchronised dual-stream RTCM3 feeder for rtkrcv replay.

Unlike tools/replay_sources.py (which paces each file by fixed byte-chunks),
this feeder paces base and rover RTCM3 files by the GPST *epoch* embedded in
their observation messages. That matters for RTK: a rover file is often ~90%
ephemeris messages (1019/1020/1042/1046), so byte-fraction is a poor proxy for
time — feeding by bytes lets base and rover drift apart by more than rtkrcv's
maxage (default 30 s), and rtkrcv then produces only a handful of solutions.

The feeder anchors on GPS MSM7 (1077) frames to recover each frame's GPST, then
releases base and rover frames on a shared, accelerated wall-clock schedule so
age-of-differential stays small and rtkrcv fixes normally.

Usage:
  python tools/rtkfeed_timed.py BASE.rtcm3 6201 ROVER.rtcm3 6202 [SPEED]

Serves BASE on its port and ROVER on its port; the app's corrections/raw_obs
collectors connect in as TCP clients. SPEED (default 30) accelerates playback:
one hour of 30 s-epoch data streams in ~2 min at 30x. See docs/replay-demo.md.
"""
from __future__ import annotations

import asyncio
import sys


def iter_frames(data: bytes):
    """Yield (msgnum, frame_bytes) for each RTCM3 frame in `data`.

    RTCM3 framing: 0xD3, 10-bit big-endian length, payload, 3-byte CRC24. The
    12-bit message number is the first 12 bits of the payload. CRC is not
    verified (rnx2rtcm output is trusted); framing is validated by length.
    """
    i, n = 0, len(data)
    while i < n:
        if data[i] != 0xD3:
            i += 1
            continue
        if i + 3 > n:
            break
        ln = ((data[i + 1] & 0x03) << 8) | data[i + 2]
        end = i + 3 + ln + 3  # preamble+len(3) + payload(ln) + CRC24(3)
        if end > n:
            break
        frame = data[i:end]
        payload = frame[3:3 + ln]
        msg = (payload[0] << 4) | (payload[1] >> 4)
        yield msg, frame
        i = end


def read_bits(payload: bytes, pos: int, length: int) -> int:
    """Read `length` bits from `payload` starting at bit offset `pos` (MSB-first)."""
    v = 0
    for k in range(length):
        byte = payload[(pos + k) >> 3]
        bit = (byte >> (7 - ((pos + k) & 7))) & 1
        v = (v << 1) | bit
    return v


def gps_tow_ms(frame: bytes) -> int:
    """DF004 GPS epoch time (ms of week): 30 bits after msg(12)+refstation(12)."""
    ln = ((frame[1] & 0x03) << 8) | frame[2]
    payload = frame[3:3 + ln]
    return read_bits(payload, 24, 30)


def build_schedule(path: str) -> list[tuple[int, bytes]]:
    """Return [(tow_ms, frame), …] for a file, tagging each frame with the tow of
    the most recent GPS MSM7 (1077) obs frame (ephemeris/other frames inherit the
    current epoch's tow; leading frames before the first obs get the first tow)."""
    data = open(path, "rb").read()
    out: list[tuple[int | None, bytes]] = []
    cur: int | None = None
    for msg, frame in iter_frames(data):
        if msg == 1077:
            cur = gps_tow_ms(frame)
        out.append((cur, frame))
    first = next((t for t, _ in out if t is not None), 0)
    return [(t if t is not None else first, fr) for t, fr in out]


async def _serve(port: int, sched, tow0: int, speed: float, t_start_ref):
    async def handler(reader, writer):
        t_start = t_start_ref()
        for tow, frame in sched:
            target = t_start + (tow - tow0) / 1000.0 / speed
            dt = target - asyncio.get_event_loop().time()
            if dt > 0:
                await asyncio.sleep(dt)
            writer.write(frame)
            await writer.drain()
        await asyncio.sleep(15)   # hold the connection briefly after one pass

    server = await asyncio.start_server(handler, "127.0.0.1", port)
    async with server:
        await server.serve_forever()


async def main() -> None:
    base_f, base_p = sys.argv[1], int(sys.argv[2])
    rover_f, rover_p = sys.argv[3], int(sys.argv[4])
    speed = float(sys.argv[5]) if len(sys.argv) > 5 else 30.0

    base_sched, rover_sched = build_schedule(base_f), build_schedule(rover_f)
    tow0 = min(min(t for t, _ in base_sched), min(t for t, _ in rover_sched))
    span = (max(t for t, _ in rover_sched) - tow0) / 1000.0
    print(f"base frames={len(base_sched)} rover frames={len(rover_sched)} "
          f"span={span:.0f}s speed={speed}x", flush=True)

    # A single shared start reference (a moment in the future) for both streams:
    # base and rover schedule against the same t_start, so per-connection timing
    # skew cannot desync them in GPST — what maxage cares about.
    t_start = asyncio.get_event_loop().time() + 2.0
    ref = lambda: t_start
    await asyncio.gather(_serve(base_p, base_sched, tow0, speed, ref),
                         _serve(rover_p, rover_sched, tow0, speed, ref))


if __name__ == "__main__":
    asyncio.run(main())
