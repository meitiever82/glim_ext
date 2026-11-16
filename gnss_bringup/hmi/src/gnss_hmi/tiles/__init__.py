"""Read-only MBTiles access for the offline imagery basemap (spec §5)."""
from __future__ import annotations

import os
import sqlite3

from gnss_hmi.tiles.manager import TileManager, describe_store

__all__ = ["TileStore", "TileManager", "open_tile_store", "describe_store"]


class TileStore:
    def __init__(self, path: str) -> None:
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(path)
        self._db = sqlite3.connect(f"file:{path}?mode=ro", uri=True,
                                   check_same_thread=False)
        # 坏文件（截断/非 SQLite）要到真正查询时才暴露；这里主动探一次，
        # 让启动阶段就能退回网格而不是等第一个瓦片请求 500。
        try:
            self._db.execute("SELECT zoom_level FROM tiles LIMIT 1").fetchone()
        except sqlite3.DatabaseError:
            self._db.close()
            raise

    def get(self, z: int, x: int, y: int) -> bytes | None:
        row = (2 ** z) - 1 - y
        cur = self._db.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, row)).fetchone()
        return bytes(cur[0]) if cur else None

    def count(self) -> int:
        return int(self._db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0])

    def close(self) -> None:
        self._db.close()


def open_tile_store(path: str) -> TileStore | None:
    """打开瓦片库；缺失或损坏都返回 None（页面据此退网格/在线底图）。"""
    try:
        return TileStore(path)
    except (FileNotFoundError, sqlite3.DatabaseError, OSError):
        return None
