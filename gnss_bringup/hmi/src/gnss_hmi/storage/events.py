"""SQLite-backed event log. Plan 2's diagnosis engine extends this table."""
from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path

_SCHEMA = """CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL,
    etype TEXT NOT NULL,
    state TEXT NOT NULL,
    detail TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_events_t ON events(t);"""

_EXTRA_COLS = (("level", "TEXT"), ("code", "TEXT"), ("t_close", "REAL"),
               ("lat", "REAL"), ("lon", "REAL"),
               ("lat_close", "REAL"), ("lon_close", "REAL"), ("peak", "TEXT"))


@dataclass(frozen=True)
class EventRow:
    id: int
    t: float
    etype: str
    state: str
    detail: str
    level: str | None = None
    code: str | None = None
    t_close: float | None = None
    lat: float | None = None
    lon: float | None = None
    lat_close: float | None = None
    lon_close: float | None = None
    peak: str | None = None


class EventStore:
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
        self._migrate()

    def _migrate(self) -> None:
        cols = {r[1] for r in self._db.execute("PRAGMA table_info(events)")}
        for col, typ in _EXTRA_COLS:
            if col not in cols:
                self._db.execute(f"ALTER TABLE events ADD COLUMN {col} {typ}")
        self._db.commit()

    def record(self, t: float, etype: str, state: str, detail: str = "",
               level: str | None = None, code: str | None = None,
               lat: float | None = None, lon: float | None = None) -> int:
        cur = self._db.execute(
            "INSERT INTO events (t, etype, state, detail, level, code, lat, lon)"
            " VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (t, etype, state, detail, level, code, lat, lon))
        self._db.commit()
        return int(cur.lastrowid)

    def query(self, since: float = 0.0) -> list[EventRow]:
        rows = self._db.execute(
            "SELECT id, t, etype, state, detail, level, code, t_close, lat, lon, lat_close, lon_close, peak"
            " FROM events WHERE t >= ? ORDER BY t",
            (since,)).fetchall()
        return [EventRow(*r) for r in rows]

    def close_event(self, event_id: int, t_close: float, lat: float | None = None,
                    lon: float | None = None, peak: str | None = None) -> None:
        self._db.execute(
            "UPDATE events SET state='closed', t_close=?, lat_close=?, lon_close=?, peak=?"
            " WHERE id=?", (t_close, lat, lon, peak, event_id))
        self._db.commit()

    def prune(self, before_t: float) -> int:
        # state != 'open' (not state='closed'): the table also holds
        # link/crash rows (corrections_link/web/rtkrcv/... with state
        # connected/disconnected/crashed/...) that never transition through
        # 'closed'. Only rows still actively open (any etype) must survive
        # retention -- everything else past the cutoff is prunable.
        cur = self._db.execute(
            "DELETE FROM events WHERE t < ? AND state != 'open'", (before_t,))
        self._db.commit()
        return cur.rowcount

    def checkpoint(self) -> None:
        """Compact the WAL file back into the db (long-running ARM disk hygiene)."""
        self._db.execute("PRAGMA wal_checkpoint(TRUNCATE)")

    def close(self) -> None:
        self._db.close()


class MemoryEventStore:
    """P0 只读模式的进程内事件表（不落盘）。

    与 EventStore 的查询表面保持一致，供 api.py / report.py 直接复用；
    进程退出即丢弃，符合 P0“无存储”约束。
    """

    def __init__(self, max_rows: int = 5000) -> None:
        self._rows: list[EventRow] = []
        self._next_id = 1
        self._max = max_rows

    def record(self, t: float, etype: str, state: str, detail: str = "",
               level: str | None = None, code: str | None = None,
               lat: float | None = None, lon: float | None = None) -> int:
        rid = self._next_id
        self._next_id += 1
        self._rows.append(EventRow(rid, t, etype, state, detail, level, code,
                                   None, lat, lon, None, None, None))
        if len(self._rows) > self._max:
            del self._rows[:len(self._rows) - self._max]
        return rid

    def query(self, since: float = 0.0) -> list[EventRow]:
        return [r for r in self._rows if r.t >= since]

    def close_event(self, event_id: int, t_close: float, lat: float | None = None,
                    lon: float | None = None, peak: str | None = None) -> None:
        for i, r in enumerate(self._rows):
            if r.id == event_id:
                self._rows[i] = EventRow(r.id, r.t, r.etype, "closed", r.detail,
                                         r.level, r.code, t_close, r.lat, r.lon,
                                         lat, lon, peak)
                return

    def prune(self, before_t: float) -> int:
        keep = [r for r in self._rows if r.t >= before_t or r.state == "open"]
        removed = len(self._rows) - len(keep)
        self._rows = keep
        return removed

    def checkpoint(self) -> None:
        pass

    def close(self) -> None:
        self._rows.clear()
