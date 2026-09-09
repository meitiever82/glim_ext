#!/usr/bin/env python3
"""tools/export_bag_to_pos.py --db 的单元测试(unittest;`python3 -m unittest tests/test_export_bag_to_pos.py`
或 `python3 -m pytest tests`)。用 sqlite3 造一个与 rtk-monitor `epochs` 表同 schema 的临时 DB。"""
import calendar
import os
import sqlite3
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import export_bag_to_pos as ex  # noqa: E402

SCHEMA = """CREATE TABLE epochs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL, src TEXT NOT NULL,
    q INTEGER, sats INTEGER, age REAL,
    lat REAL, lon REAL, alt REAL,
    sde REAL, sdn REAL, sdu REAL,
    ratio REAL, heading REAL, speed REAL, sats_json TEXT
);"""
COLS = ("t", "src", "q", "sats", "age", "lat", "lon", "alt", "sde", "sdn", "sdu", "ratio", "heading", "speed",
        "sats_json")

T0 = 1757300000.0   # 2025-09-08 03:33:20 UTC


def parse_pos(path):
    """与 tools/make_synthetic_pos.py 同风格的纯 Python .pos 解析:返回 (header_lines, rows),
    rows 每行为 (stamp_utc, [14 列字符串])。"""
    headers, rows = [], []
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            if line[0] == "%":
                headers.append(line.rstrip("\n"))
                continue
            c = line.split()
            y, mo, d = (int(x) for x in c[0].split("/"))
            hh, mm, ss = c[1].split(":")
            stamp = calendar.timegm((y, mo, d, int(hh), int(mm), 0)) + float(ss)
            rows.append((stamp, c))
    return headers, rows


class ExportDbTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.db = os.path.join(self.tmp.name, "epochs.db")
        self.out = os.path.join(self.tmp.name, "out.pos")
        self.conn = sqlite3.connect(self.db)
        self.conn.executescript(SCHEMA)

    def tearDown(self):
        self.conn.close()
        self.tmp.cleanup()

    def insert(self, **kw):
        row = {c: None for c in COLS}
        row.update(kw)
        self.conn.execute(f"INSERT INTO epochs ({','.join(COLS)}) VALUES ({','.join('?' * len(COLS))})",
                          [row[c] for c in COLS])
        self.conn.commit()

    def run_cli(self, *extra):
        rc = ex.main(["--db", self.db, "--out", self.out, *extra])
        self.assertEqual(rc, 0)
        return parse_pos(self.out)

    # --- CGI-610 satellite_status → RTKLIB Q ---
    def test_cgi610_status_mapping(self):
        for s, q in ((4, 1), (8, 1), (5, 2), (9, 2), (2, 4), (7, 4), (1, 5), (6, 5), (3, 5), (0, 0)):
            self.assertEqual(ex.cgi610_status_to_q(s), q, f"status {s}")
        self.assertEqual(ex.cgi610_status_to_q(None), 0)
        self.assertEqual(ex.cgi610_status_to_q(11), 0)   # 未知码 → 无解

    def test_rtkrcv_q_passthrough(self):
        for q in (1, 2, 4, 5):
            self.assertEqual(ex.rtkrcv_q_to_q(q), q)
        self.assertEqual(ex.rtkrcv_q_to_q(None), 0)
        self.assertEqual(ex.rtkrcv_q_to_q(0), 0)
        self.assertEqual(ex.rtkrcv_q_to_q(3), 0)   # SBAS → gnss_core 侧为 NONE,写 0

    # --- 列顺序:DB 是 sde sdn sdu,.pos 是 sdn sde sdu;t 为 UTC ---
    def test_column_order_and_values(self):
        self.insert(t=T0 + 0.25, src="gpchc", q=4, sats=23, age=1.5, lat=44.5, lon=90.28, alt=617.3,
                    sde=0.011, sdn=0.022, sdu=0.033, ratio=12.5, heading=90.0, speed=1.0)
        headers, rows = self.run_cli("--src", "gpchc")
        self.assertTrue(any("time=UTC" in h for h in headers))
        self.assertEqual(len(rows), 1)
        stamp, c = rows[0]
        self.assertEqual(len(c), 15)   # date + time + 13 列 = read_pos 的 "14 列"(date/time 合一)
        self.assertAlmostEqual(stamp, T0 + 0.25, places=3)
        self.assertEqual(c[0], "2025/09/08")
        self.assertAlmostEqual(float(c[2]), 44.5, places=8)
        self.assertAlmostEqual(float(c[3]), 90.28, places=8)
        self.assertAlmostEqual(float(c[4]), 617.3, places=3)
        self.assertEqual(int(c[5]), 1)      # Q: 610 status 4 → fix
        self.assertEqual(int(c[6]), 23)     # ns
        self.assertAlmostEqual(float(c[7]), 0.022, places=4)   # sdn
        self.assertAlmostEqual(float(c[8]), 0.011, places=4)   # sde
        self.assertAlmostEqual(float(c[9]), 0.033, places=4)   # sdu
        self.assertAlmostEqual(float(c[13]), 1.5, places=2)    # age
        self.assertAlmostEqual(float(c[14]), 12.5, places=1)   # ratio

    def test_null_optional_columns_written_as_zero(self):
        self.insert(t=T0, src="rtkrcv", q=2, sats=None, age=None, lat=44.5, lon=90.28, alt=617.0,
                    sde=None, sdn=None, sdu=None, ratio=None)
        _, rows = self.run_cli("--src", "rtkrcv")
        self.assertEqual(len(rows), 1)
        c = rows[0][1]
        self.assertEqual(int(c[5]), 2)
        self.assertEqual(int(c[6]), 0)
        for i in (7, 8, 9, 13, 14):
            self.assertEqual(float(c[i]), 0.0)

    # --- src 过滤 + 时间窗 ---
    def test_src_filter_and_time_window(self):
        for i in range(5):
            self.insert(t=T0 + i, src="can", q=4, sats=20, lat=44.5, lon=90.28, alt=617.0)
        self.insert(t=T0 + 2, src="gpchc", q=4, sats=20, lat=44.5, lon=90.28, alt=617.0)
        _, rows = self.run_cli("--src", "can")
        self.assertEqual(len(rows), 5)
        _, rows = self.run_cli("--src", "can", "--t0", str(T0 + 1), "--t1", str(T0 + 3))
        self.assertEqual([round(r[0] - T0) for r in rows], [1, 2, 3])   # 闭区间
        _, rows = self.run_cli("--src", "can", "--t0", str(T0 + 4))
        self.assertEqual(len(rows), 1)
        _, rows = self.run_cli("--src", "can", "--t1", str(T0 + 0.5))
        self.assertEqual(len(rows), 1)

    def test_output_sorted_by_time(self):
        for i in (3, 1, 2):
            self.insert(t=T0 + i, src="can", q=4, sats=20, lat=44.5, lon=90.28, alt=617.0)
        _, rows = self.run_cli("--src", "can")
        self.assertEqual([round(r[0] - T0) for r in rows], [1, 2, 3])

    # --- NOFIX:默认跳过,--keep-nofix 保留并写 Q=0 ---
    def test_nofix_skipped_by_default(self):
        self.insert(t=T0, src="can", q=0, sats=3, lat=44.5, lon=90.28, alt=617.0)
        self.insert(t=T0 + 1, src="can", q=4, sats=20, lat=44.5, lon=90.28, alt=617.0)
        self.insert(t=T0 + 2, src="can", q=None, sats=20, lat=44.5, lon=90.28, alt=617.0)
        _, rows = self.run_cli("--src", "can")
        self.assertEqual(len(rows), 1)
        self.assertEqual(int(rows[0][1][5]), 1)

    def test_keep_nofix_writes_q0(self):
        self.insert(t=T0, src="can", q=0, sats=3, lat=44.5, lon=90.28, alt=617.0)
        self.insert(t=T0 + 1, src="can", q=4, sats=20, lat=44.5, lon=90.28, alt=617.0)
        _, rows = self.run_cli("--src", "can", "--keep-nofix")
        self.assertEqual([int(r[1][5]) for r in rows], [0, 1])

    def test_rows_without_position_skipped(self):
        self.insert(t=T0, src="rtkrcv", q=1, sats=20, lat=None, lon=None, alt=None)
        self.insert(t=T0 + 1, src="rtkrcv", q=1, sats=20, lat=44.5, lon=90.28, alt=617.0)
        _, rows = self.run_cli("--src", "rtkrcv")
        self.assertEqual(len(rows), 1)

    def test_missing_table_raises(self):
        empty = os.path.join(self.tmp.name, "empty.db")
        sqlite3.connect(empty).close()
        with self.assertRaises(RuntimeError):
            ex.load_epochs_from_db(empty, "can")


if __name__ == "__main__":
    unittest.main()
