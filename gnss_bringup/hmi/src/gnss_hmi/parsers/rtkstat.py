"""Parse rtkrcv solution-status $SAT lines; track cycle-slip counts in a window."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SatStat:
    tow: float
    sat: str
    az: float
    el: float
    resp: float       # pseudorange residual (m)
    snr: float        # dBHz
    valid: bool
    slipc: int
    rejc: int


def parse_sat_line(line: str) -> SatStat | None:
    if not line.startswith("$SAT,"):
        return None
    p = line.strip().split(",")
    if len(p) < 17:
        return None
    try:
        return SatStat(tow=float(p[2]), sat=p[3], az=float(p[5]), el=float(p[6]),
                       resp=float(p[7]), snr=float(p[10]), valid=p[9] == "1",
                       slipc=int(p[15]), rejc=int(p[16]))
    except ValueError:
        return None


class StatEpochAccumulator:
    """Group a stream of rtkrcv $SAT lines into the latest epoch's per-satellite
    data. One entry per satellite (first frequency wins). `sats` are skyplot
    dicts ({sat,az,el,snr,used}); `satstats` are the SatStat objects the
    diagnosis multipath rule consumes. Both hold the last *complete* epoch until
    a new one starts accumulating, so a consumer never sees an empty flicker at
    an epoch boundary. Feed lines as they arrive (across reads); reset() when
    switching to a fresh .stat file."""

    def __init__(self) -> None:
        self.sats: list[dict] = []
        self.satstats: list[SatStat] = []
        self._tow: float | None = None
        self._cur_dicts: list[dict] = []
        self._cur_stats: list[SatStat] = []
        self._seen: set[str] = set()

    def reset(self) -> None:
        self.__init__()

    def feed(self, line: str, on_slip=None) -> None:
        st = parse_sat_line(line)
        if st is None:
            return
        if self._tow is not None and st.tow != self._tow:
            # new epoch begins; the public lists keep pointing at the completed
            # epoch until the new one accumulates its first satellite
            self._cur_dicts, self._cur_stats, self._seen = [], [], set()
        self._tow = st.tow
        if st.sat in self._seen:      # one point per sat (skip 2nd frequency)
            return
        self._seen.add(st.sat)
        self._cur_dicts.append({"sat": st.sat, "az": st.az, "el": st.el,
                                "snr": st.snr, "used": st.valid})
        self._cur_stats.append(st)
        if on_slip is not None:
            on_slip(st.sat, st.slipc)
        self.sats = self._cur_dicts       # live refs; grow as the epoch fills
        self.satstats = self._cur_stats


class SlipWindow:
    """Count cycle-slip increments across all satellites within a sliding window."""

    def __init__(self, window_s: float = 30.0) -> None:
        self._window = window_s
        self._last: dict[str, int] = {}
        self._hits: list[tuple[float, int]] = []   # (t, delta)

    def feed(self, t: float, sat: str, slipc: int) -> None:
        prev = self._last.get(sat)
        self._last[sat] = slipc
        if prev is not None and slipc > prev:
            self._hits.append((t, slipc - prev))

    def count(self, now: float) -> int:
        cutoff = now - self._window
        self._hits = [(t, d) for t, d in self._hits if t >= cutoff]
        return sum(d for _, d in self._hits)
