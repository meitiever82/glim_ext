"""Parse RTKLIB llh-format solution lines from rtkrcv's output stream.

Column order (out-outhead off): date time lat lon height Q ns sdn sde sdu
sdne sdeu sdun age ratio. Timestamps are GPST parsed as UTC (constant ~18 s
offset; only time differences are consumed downstream).
"""
from __future__ import annotations

import datetime
from dataclasses import dataclass


@dataclass(frozen=True)
class RtkSolution:
    t: float
    lat: float
    lon: float
    alt: float
    q: int          # RTKLIB: 1=fix 2=float 4=dgps 5=single
    ns: int
    sdn: float
    sde: float
    sdu: float
    age: float
    ratio: float


def parse_llh_solution(line: str) -> RtkSolution | None:
    line = line.strip()
    if not line or line.startswith("%"):
        return None
    f = line.split()
    if len(f) < 15:
        return None
    try:
        dt = datetime.datetime.strptime(f[0] + " " + f[1], "%Y/%m/%d %H:%M:%S.%f")
        return RtkSolution(
            t=dt.replace(tzinfo=datetime.timezone.utc).timestamp(),
            lat=float(f[2]), lon=float(f[3]), alt=float(f[4]),
            q=int(f[5]), ns=int(f[6]),
            sdn=float(f[7]), sde=float(f[8]), sdu=float(f[9]),
            age=float(f[13]), ratio=float(f[14]),
        )
    except ValueError:
        return None
