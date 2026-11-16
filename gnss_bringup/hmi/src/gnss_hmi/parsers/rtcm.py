"""Incremental RTCM3 frame splitter and message-1005 station coordinates."""
from __future__ import annotations

from dataclasses import dataclass

CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF


@dataclass(frozen=True)
class RtcmMessage:
    msg_type: int
    payload: bytes
    raw: bytes


class RtcmFramer:
    """Feed arbitrary byte chunks; get complete CRC-checked RTCM3 messages."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[RtcmMessage]:
        self._buf.extend(data)
        out: list[RtcmMessage] = []
        while True:
            start = self._buf.find(b"\xd3")
            if start < 0:
                self._buf.clear()
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < 6:
                break
            length = ((self._buf[1] & 0x03) << 8) | self._buf[2]
            total = 3 + length + 3
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                payload = frame[3:-3]
                msg_type = (payload[0] << 4) | (payload[1] >> 4)
                out.append(RtcmMessage(msg_type, payload, frame))
                del self._buf[:total]
            else:
                self.crc_errors += 1
                del self._buf[:1]
        return out


def _get_bits(data: bytes, start: int, length: int) -> int:
    value = 0
    for i in range(start, start + length):
        value = (value << 1) | ((data[i // 8] >> (7 - i % 8)) & 1)
    return value


def _get_sbits(data: bytes, start: int, length: int) -> int:
    v = _get_bits(data, start, length)
    return v - (1 << length) if v & (1 << (length - 1)) else v


def parse_1005(payload: bytes) -> tuple[float, float, float]:
    """Return base-station ECEF (x, y, z) in meters from a 1005 payload."""
    x = _get_sbits(payload, 34, 38) * 1e-4
    y = _get_sbits(payload, 74, 38) * 1e-4
    z = _get_sbits(payload, 114, 38) * 1e-4
    return x, y, z
