"""Parse Huace $GPCHC integrated-navigation sentences (CGI-610 'satnav data')."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GpchcEpoch:
    week: int
    tow: float
    heading: float
    pitch: float
    roll: float
    lat: float
    lon: float
    alt: float
    ve: float
    vn: float
    vu: float
    speed: float
    nsv1: int
    nsv2: int
    sat_status: int
    sys_state: int
    diff_age: float


class LineFramer:
    """Reassemble a TCP byte stream into complete text lines."""

    def __init__(self, max_buf: int = 65536) -> None:
        self._buf = b""
        self._max = max_buf
        self.overflows = 0

    def feed(self, data: bytes) -> list[str]:
        self._buf += data
        if len(self._buf) > self._max and b"\n" not in self._buf:
            self._buf = b""
            self.overflows += 1
            return []
        *lines, self._buf = self._buf.split(b"\n")
        return [ln.strip().decode("ascii", "replace") for ln in lines if ln.strip()]


def parse_gpchc(line: str) -> GpchcEpoch | None:
    if not line.startswith("$GPCHC,") or "*" not in line:
        return None
    body, cs_str = line[1:].rsplit("*", 1)
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    try:
        if cs != int(cs_str, 16):
            return None
        f = body.split(",")
        status = int(f[21], 16)
        return GpchcEpoch(
            week=int(f[1]), tow=float(f[2]),
            heading=float(f[3]), pitch=float(f[4]), roll=float(f[5]),
            lat=float(f[12]), lon=float(f[13]), alt=float(f[14]),
            ve=float(f[15]), vn=float(f[16]), vu=float(f[17]), speed=float(f[18]),
            nsv1=int(f[19]), nsv2=int(f[20]),
            sat_status=(status >> 4) & 0xF, sys_state=status & 0xF,
            diff_age=float(f[22]),
        )
    except (IndexError, ValueError):
        return None
