"""SQLite epoch summaries (1 Hz per source) plus base-station history and a KV table.

`q` semantics depend on `src`: for "rtkrcv" it is the RTKLIB quality flag
(1=fix 2=float 4=dgps 5=single); for "gpchc"/"can" it is the CGI-610
satellite-status nibble (4=RTK fixed with heading, 5=float, ...).
"""
from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path

_SCHEMA = """CREATE TABLE IF NOT EXISTS epochs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL, src TEXT NOT NULL,
    q INTEGER, sats INTEGER, age REAL,
    lat REAL, lon REAL, alt REAL,
    sde REAL, sdn REAL, sdu REAL,
    ratio REAL, heading REAL, speed REAL, sats_json TEXT
);
CREATE INDEX IF NOT EXISTS idx_epochs_t ON epochs(t);
CREATE INDEX IF NOT EXISTS idx_epochs_src_t ON epochs(src, t);
CREATE TABLE IF NOT EXISTS kv (k TEXT PRIMARY KEY, v TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS base_station (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_base_t ON base_station(t);"""

_COLS = ("t", "src", "q", "sats", "age", "lat", "lon", "alt",
         "sde", "sdn", "sdu", "ratio", "heading", "speed", "sats_json")


@dataclass(frozen=True)
class Epoch:
    t: float
    src: str
    q: int | None = None
    sats: int | None = None
    age: float | None = None
    lat: float | None = None
    lon: float | None = None
    alt: float | None = None
    sde: float | None = None
    sdn: float | None = None
    sdu: float | None = None
    ratio: float | None = None
    heading: float | None = None
    speed: float | None = None
    sats_json: str | None = None


class EpochStore:
    def __init__(self, db_path: str | Path) -> None:
        # check_same_thread=False: production access stays on the event loop
        # thread (async handlers), but test clients (TestClient blocking
        # portal) touch the store from another thread; sqlite3 is compiled
        # serialized (threadsafety=3) so sharing is safe.
        self._db = sqlite3.connect(db_path, check_same_thread=False)
        self._db.executescript(_SCHEMA)
        self._db.commit()
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA busy_timeout=5000")

    def add(self, e: Epoch) -> int:
        vals = [getattr(e, c) for c in _COLS]
        cur = self._db.execute(
            f"INSERT INTO epochs ({','.join(_COLS)}) VALUES ({','.join('?' * len(_COLS))})",
            vals)
        self._db.commit()
        return int(cur.lastrowid)

    def _row_to_epoch(self, row) -> Epoch:
        return Epoch(**dict(zip(_COLS, row)))

    def latest(self, src: str) -> Epoch | None:
        row = self._db.execute(
            f"SELECT {','.join(_COLS)} FROM epochs WHERE src=? ORDER BY t DESC LIMIT 1",
            (src,)).fetchone()
        return self._row_to_epoch(row) if row else None

    def query(self, src: str, t0: float, t1: float) -> list[Epoch]:
        rows = self._db.execute(
            f"SELECT {','.join(_COLS)} FROM epochs WHERE src=? AND t>=? AND t<=? ORDER BY t",
            (src, t0, t1)).fetchall()
        return [self._row_to_epoch(r) for r in rows]

    def query_last(self, src: str, t0: float, t1: float, limit: int) -> list[Epoch]:
        """Newest `limit` rows in [t0, t1], oldest-first. Pushes the "give me
        the newest N" selection into SQL (ORDER BY t DESC LIMIT ?) instead of
        a caller running the unbounded `query()` and slicing [-limit:] in
        Python, which materializes every row in the range just to discard
        all but the tail."""
        rows = self._db.execute(
            f"SELECT {','.join(_COLS)} FROM epochs WHERE src=? AND t>=? AND t<=?"
            " ORDER BY t DESC LIMIT ?",
            (src, t0, t1, limit)).fetchall()
        return [self._row_to_epoch(r) for r in reversed(rows)]

    def kv_get(self, k: str) -> str | None:
        row = self._db.execute("SELECT v FROM kv WHERE k=?", (k,)).fetchone()
        return row[0] if row else None

    def kv_set(self, k: str, v: str) -> None:
        self._db.execute(
            "INSERT INTO kv (k, v) VALUES (?, ?) ON CONFLICT(k) DO UPDATE SET v=excluded.v",
            (k, v))
        self._db.commit()

    def add_base(self, t: float, x: float, y: float, z: float) -> None:
        self._db.execute("INSERT INTO base_station (t, x, y, z) VALUES (?, ?, ?, ?)",
                         (t, x, y, z))
        self._db.commit()

    def base_history(self, since: float = 0.0) -> list[tuple[float, float, float, float]]:
        return self._db.execute(
            "SELECT t, x, y, z FROM base_station WHERE t>=? ORDER BY t",
            (since,)).fetchall()

    def prune(self, before_t: float) -> int:
        cur = self._db.execute("DELETE FROM epochs WHERE t < ?", (before_t,))
        n = cur.rowcount
        n += self._db.execute("DELETE FROM base_station WHERE t < ?", (before_t,)).rowcount
        self._db.commit()
        return n

    def checkpoint(self) -> None:
        """Compact the WAL file back into the db (long-running ARM disk hygiene)."""
        self._db.execute("PRAGMA wal_checkpoint(TRUNCATE)")

    def close(self) -> None:
        self._db.close()
