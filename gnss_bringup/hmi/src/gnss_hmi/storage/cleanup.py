"""Delete oldest day directories by retention age or disk watermark."""
from __future__ import annotations

import datetime
import re
import shutil
from pathlib import Path


def cleanup_logs(root: Path, retention_days: int, watermark_pct: float,
                 disk_usage=shutil.disk_usage,
                 today: datetime.date | None = None) -> list[Path]:
    today = today or datetime.date.today()
    day_dirs = sorted(d for d in Path(root).iterdir()
                      if d.is_dir() and re.fullmatch(r"\d{8}", d.name))
    deleted: list[Path] = []
    for d in day_dirs:
        d_date = datetime.datetime.strptime(d.name, "%Y%m%d").date()
        if d_date >= today:
            break  # never delete today's (or a future-dated) directory
        u = disk_usage(root)
        over_watermark = u.used / u.total * 100 > watermark_pct
        too_old = (today - d_date).days > retention_days
        if not (too_old or over_watermark):
            break
        shutil.rmtree(d)
        deleted.append(d)
    return deleted
