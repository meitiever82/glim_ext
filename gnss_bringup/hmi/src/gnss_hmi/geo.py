"""Shared geodesy helpers."""
from __future__ import annotations

import math


def horiz_dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle horizontal distance in metres (haversine)."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin(math.radians(lat2 - lat1) / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))
