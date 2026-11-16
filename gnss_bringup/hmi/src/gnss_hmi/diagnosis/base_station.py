"""Learn the base station's ECEF baseline from 1005 messages; report offsets."""
from __future__ import annotations

import logging
import math
import statistics

from gnss_hmi.storage.epochs import EpochStore

_KV_KEY = "base_xyz"
_logger = logging.getLogger(__name__)


class BaseStationMonitor:
    def __init__(self, store: EpochStore, warmup_s: float = 600.0) -> None:
        self._store = store
        self._warmup = warmup_s
        self._samples: list[tuple[float, float, float, float]] = []
        self._last_hist: tuple[float, float, float] | None = None
        self._baseline: tuple[float, float, float] | None = None
        stored = store.kv_get(_KV_KEY)
        if stored:
            try:
                x, y, z = (float(v) for v in stored.split(","))
            except ValueError:
                # Corrupt kv row (e.g. truncated write during a power loss
                # under docker restart:always) must not brick startup --
                # fall back to a fresh warmup instead of crashing.
                _logger.exception("corrupt base_xyz kv value %r; re-warming up", stored)
            else:
                self._baseline = (x, y, z)

    def feed(self, t: float, x: float, y: float, z: float) -> float | None:
        if self._last_hist is None or any(
                abs(a - b) > 1e-3 for a, b in zip((x, y, z), self._last_hist)):
            self._store.add_base(t, x, y, z)
            self._last_hist = (x, y, z)
        if self._baseline is None:
            self._samples.append((t, x, y, z))
            if t - self._samples[0][0] >= self._warmup:
                bx = statistics.median(s[1] for s in self._samples)
                by = statistics.median(s[2] for s in self._samples)
                bz = statistics.median(s[3] for s in self._samples)
                self._set_baseline(bx, by, bz)
            else:
                return None
        assert self._baseline is not None
        bx, by, bz = self._baseline
        return math.dist((x, y, z), (bx, by, bz))

    def reset(self, t: float, x: float, y: float, z: float) -> None:
        """Operator-confirmed baseline update (surfaced in the UI in Plan 3)."""
        self._set_baseline(x, y, z)
        self._store.add_base(t, x, y, z)
        self._last_hist = (x, y, z)

    def _set_baseline(self, x: float, y: float, z: float) -> None:
        self._baseline = (x, y, z)
        self._store.kv_set(_KV_KEY, f"{x:.4f},{y:.4f},{z:.4f}")
        self._samples.clear()
