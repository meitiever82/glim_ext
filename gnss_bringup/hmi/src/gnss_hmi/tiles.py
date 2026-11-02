"""Read-only MBTiles access for the offline imagery basemap (spec §5)."""
from __future__ import annotations

import os
import sqlite3


class TileStore:
    def __init__(self, path: str) -> None:
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(path)
        self._db = sqlite3.connect(f"file:{path}?mode=ro", uri=True,
                                   check_same_thread=False)

    def get(self, z: int, x: int, y: int) -> bytes | None:
        row = (2 ** z) - 1 - y
        cur = self._db.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, row)).fetchone()
        return bytes(cur[0]) if cur else None

    def close(self) -> None:
        self._db.close()
