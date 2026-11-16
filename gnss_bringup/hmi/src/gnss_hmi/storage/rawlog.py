"""Hourly-rotated raw byte-stream writer with a JSONL sidecar index.

The raw file is byte-identical to the wire stream so it can be fed directly
to RTKLIB tools (convbin, rnx2rtkp) offline.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import IO


class RawLogWriter:
    def __init__(self, root: Path, stream: str, ext: str = "bin", clock=time.time,
                 index: bool = True, flush_every: int = 1) -> None:
        self._root = Path(root)
        self._stream = stream
        self._ext = ext
        self._clock = clock
        self._index = index
        self._flush_every = max(1, flush_every)
        self._hour_key: str | None = None
        self._file: IO[bytes] | None = None
        self._idx: IO[str] | None = None
        self._pending = 0

    def append(self, data: bytes, msg_type: int | str | None = None) -> None:
        t = self._clock()
        hour_key = time.strftime("%Y%m%d_%H", time.localtime(t))
        if hour_key != self._hour_key:
            self._rotate(hour_key)
        assert self._file is not None
        off = self._file.tell()
        self._file.write(data)
        if self._idx is not None:
            self._idx.write(json.dumps(
                {"t": round(t, 3), "type": msg_type, "off": off, "len": len(data)}) + "\n")
        self._pending += 1
        if self._pending >= self._flush_every:
            self._file.flush()
            if self._idx is not None:
                self._idx.flush()
            self._pending = 0

    def _rotate(self, hour_key: str) -> None:
        self.close()
        day = hour_key[:8]
        d = self._root / day
        d.mkdir(parents=True, exist_ok=True)
        base = d / f"{self._stream}_{hour_key}"
        self._file = open(f"{base}.{self._ext}", "ab")
        if self._index:
            self._idx = open(f"{base}.idx.jsonl", "a")
        self._hour_key = hour_key
        self._pending = 0

    def close(self) -> None:
        for f in (self._file, self._idx):
            if f is not None:
                f.close()
        self._file = self._idx = None
        self._hour_key = None
        self._pending = 0
