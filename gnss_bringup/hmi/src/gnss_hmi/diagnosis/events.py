"""Open/close diagnosis events with close hysteresis (spec §4.3)."""
from __future__ import annotations

from typing import Callable

from gnss_hmi.diagnosis.rules import Verdict
from gnss_hmi.storage.events import EventStore

_OPEN_LEVELS = {"warning", "serious", "critical"}
OnTransition = Callable[[str, Verdict, float], None]


class EventMachine:
    def __init__(self, store: EventStore, close_hysteresis_s: float = 10.0,
                 on_transition: OnTransition | None = None) -> None:
        self._store = store
        self._hyst = close_hysteresis_s
        self._cb = on_transition
        self._open_id: int | None = None
        self._open_code: str | None = None
        self._open_verdict: Verdict | None = None
        self._ok_since: float | None = None
        self._peak: dict[str, float] = {}
        self._last_pos: tuple | None = None

    def update(self, t: float, verdict: Verdict,
               lat: float | None = None, lon: float | None = None,
               metrics: dict[str, float] | None = None) -> None:
        active = verdict.level in _OPEN_LEVELS
        if active:
            self._ok_since = None
            if self._open_code == verdict.code:
                # Event already open for this code; accumulate metrics and position
                if metrics:
                    for k, v in metrics.items():
                        if k.endswith("_min"):
                            if k not in self._peak or v < self._peak[k]:
                                self._peak[k] = v
                        elif abs(v) > abs(self._peak.get(k, 0.0)):
                            self._peak[k] = v
                if lat is not None:
                    self._last_pos = (lat, lon)
                return
            if self._open_id is not None:            # different condition: close old
                self._close(t)
            # Open new event; clear accumulators and record open
            self._peak = {}
            self._last_pos = None
            self._open_id = self._store.record(
                t, "diagnosis", "open", verdict.message,
                level=verdict.level, code=verdict.code, lat=lat, lon=lon)
            self._open_code = verdict.code
            self._open_verdict = verdict
            # Accumulate first update's metrics and position
            if metrics:
                for k, v in metrics.items():
                    if k.endswith("_min"):
                        if k not in self._peak or v < self._peak[k]:
                            self._peak[k] = v
                    elif abs(v) > abs(self._peak.get(k, 0.0)):
                        self._peak[k] = v
            if lat is not None:
                self._last_pos = (lat, lon)
            if self._cb:
                self._cb("open", verdict, t)
            return
        # ok/info: close after hysteresis
        if self._open_id is None:
            return
        if self._ok_since is None:
            self._ok_since = t
            return
        if t - self._ok_since >= self._hyst:
            self._close(t)

    def _close(self, t: float) -> None:
        assert self._open_id is not None and self._open_verdict is not None
        event_id, verdict = self._open_id, self._open_verdict
        # Capture peak and position before state reset (for exception safety)
        import json
        peak = json.dumps(self._peak) if self._peak else None
        pos = self._last_pos or (None, None)
        # Reset state before store call (exception safety: state is clean before any callback)
        self._open_id = self._open_code = self._open_verdict = None
        self._ok_since = None
        self._peak = {}
        self._last_pos = None
        # Store close event with peak and position
        self._store.close_event(event_id, t, lat=pos[0], lon=pos[1], peak=peak)
        if self._cb:
            self._cb("close", verdict, t)
