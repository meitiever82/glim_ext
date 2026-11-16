"""Incremental decoder for CGI-610 CAN 2.0 output (little-endian, verified on real logs).

A 50 Hz cycle starts at ID 0x320 (time frame); the cycle is emitted when the
next 0x320 arrives and all required IDs were seen.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

_NEED = {0x320, 0x321, 0x322, 0x323, 0x325, 0x326, 0x327, 0x328,
         0x329, 0x32A, 0x32B, 0x32C, 0x32D, 0x32E}


def _s20(v: int) -> int:
    return v - (1 << 20) if v & (1 << 19) else v


def _three_s20(b: bytes, scale: float) -> tuple[float, float, float]:
    u = int.from_bytes(b, "little")
    return (_s20(u & 0xFFFFF) * scale,
            _s20((u >> 20) & 0xFFFFF) * scale,
            _s20((u >> 40) & 0xFFFFF) * scale)


@dataclass(frozen=True)
class NavCycle:
    host_time: float
    week: int
    tow: float
    sys_state: int
    sats_used: int
    sat_status: int
    sats2_used: int
    diff_age: float
    lat: float
    lon: float
    alt: float
    pos_sigma: tuple[float, float, float]      # E, N, U (m)
    vel: tuple[float, float, float, float]     # E, N, U, total (m/s)
    vel_sigma: tuple[float, float, float, float]
    heading: float
    pitch: float
    roll: float
    att_sigma: tuple[float, float, float]      # heading, pitch, roll (deg)
    gyro: tuple[float, float, float]           # dps
    accel: tuple[float, float, float]          # g


class Cgi610Assembler:
    def __init__(self) -> None:
        self._cur: dict[int, object] | None = None
        self.incomplete = 0

    def feed(self, can_id: int, data: bytes, host_time: float) -> NavCycle | None:
        if can_id == 0x320:
            done = self._flush()
            self._cur = {0x320: (struct.unpack("<H", data[0:2])[0],
                                 struct.unpack("<I", data[2:6])[0] * 0.001),
                         "host": host_time}
            return done
        if self._cur is None:
            return None
        c = self._cur
        if can_id == 0x321:
            c[can_id] = _three_s20(data, 0.01)
        elif can_id in (0x322, 0x329):
            c[can_id] = _three_s20(data, 0.0001)
        elif can_id == 0x323:
            c[can_id] = (data[0], data[1], data[2], data[3],
                         struct.unpack("<H", data[4:6])[0] * 0.01)
        elif can_id == 0x325:
            c[can_id] = struct.unpack("<i", data[0:4])[0] * 0.001
        elif can_id == 0x326:
            c[can_id] = _three_s20(data, 0.0001)
        elif can_id == 0x327:
            c[can_id] = tuple(x * 0.01 for x in struct.unpack("<4h", data))
        elif can_id == 0x328:
            c[can_id] = tuple(x * 0.001 for x in struct.unpack("<4H", data))
        elif can_id == 0x32A:
            hd = struct.unpack("<H", data[0:2])[0] * 0.01
            pt, rl = struct.unpack("<2h", data[2:6])
            c[can_id] = (hd, pt * 0.01, rl * 0.01)
        elif can_id == 0x32B:
            c[can_id] = _three_s20(data, 0.0001)
        elif can_id == 0x32C:
            c[can_id] = _three_s20(data, 0.01)
        elif can_id in (0x32D, 0x32E):
            c[can_id] = struct.unpack("<q", data)[0] * 1e-8
        return None

    def _flush(self) -> NavCycle | None:
        c, self._cur = self._cur, None
        if c is None:
            return None
        if not _NEED.issubset(c.keys()):
            self.incomplete += 1
            return None
        week, tow = c[0x320]
        st = c[0x323]
        hd, pt, rl = c[0x32A]
        return NavCycle(
            host_time=c["host"], week=week, tow=tow,
            sys_state=st[0], sats_used=st[1], sat_status=st[2],
            sats2_used=st[3], diff_age=st[4],
            lat=c[0x32E], lon=c[0x32D], alt=c[0x325],
            pos_sigma=c[0x326], vel=c[0x327], vel_sigma=c[0x328],
            heading=hd, pitch=pt, roll=rl, att_sigma=c[0x32B],
            gyro=c[0x321], accel=c[0x322],
        )
