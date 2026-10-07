#!/usr/bin/env python3
"""tools/field_replay/candump_log.py 的单元测试(`python3 -m unittest`)。"""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import candump_log as cl  # noqa: E402

SEG = "/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748"


class ParseLineTest(unittest.TestCase):
    def test_standard_frame(self):
        rec = cl.parse_line("(1789462171.002222) can7 320#8409DCC7310C0000")
        self.assertIsNotNone(rec)
        self.assertAlmostEqual(rec.t, 1789462171.002222, places=6)
        self.assertEqual(rec.iface, "can7")
        self.assertEqual(rec.can_id, 0x320)
        self.assertEqual(rec.data, bytes.fromhex("8409DCC7310C0000"))

    def test_extended_frame_id_8_hex_digits(self):
        rec = cl.parse_line("(1789462171.100000) can7 18FEF100#0011223344556677")
        self.assertIsNotNone(rec)
        self.assertEqual(rec.can_id, 0x18FEF100)
        self.assertEqual(rec.data, bytes.fromhex("0011223344556677"))

    def test_zero_length_data_is_allowed(self):
        rec = cl.parse_line("(1789462171.100000) can7 320#")
        self.assertIsNotNone(rec)
        self.assertEqual(rec.data, b"")

    def test_empty_line_returns_none(self):
        self.assertIsNone(cl.parse_line(""))
        self.assertIsNone(cl.parse_line("\n"))
        self.assertIsNone(cl.parse_line("   "))

    def test_missing_hash_returns_none(self):
        self.assertIsNone(cl.parse_line("(1789462171.002222) can7 320"))

    def test_missing_timestamp_parens_returns_none(self):
        self.assertIsNone(cl.parse_line("1789462171.002222 can7 320#8409DCC7310C0000"))

    def test_non_hex_data_returns_none(self):
        self.assertIsNone(cl.parse_line("(1789462171.002222) can7 320#ZZ"))

    def test_non_numeric_timestamp_returns_none(self):
        self.assertIsNone(cl.parse_line("(abc) can7 320#8409DCC7310C0000"))


class FirstTimestampTest(unittest.TestCase):
    def test_skips_leading_bad_lines(self):
        with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as f:
            f.write("not a valid line\n")
            f.write("\n")
            f.write("(1789462171.002222) can7 320#8409DCC7310C0000\n")
            f.write("(1789462171.002532) can7 321#FBFF5FFFFFFCFF0F\n")
            path = f.name
        try:
            self.assertAlmostEqual(cl.first_timestamp(path), 1789462171.002222, places=6)
        finally:
            os.remove(path)

    def test_no_valid_line_raises(self):
        with tempfile.NamedTemporaryFile("w", suffix=".log", delete=False) as f:
            f.write("garbage\n")
            path = f.name
        try:
            with self.assertRaises(ValueError):
                cl.first_timestamp(path)
        finally:
            os.remove(path)


@unittest.skipUnless(os.path.isfile(os.path.join(SEG, "raw", "can7.candump.log")),
                      f"数据段不存在,跳过真实文件用例:{SEG}")
class RealFileTest(unittest.TestCase):
    def test_first_timestamp_and_first_line_parse(self):
        path = os.path.join(SEG, "raw", "can7.candump.log")
        t0 = cl.first_timestamp(path)
        with open(path) as f:
            first_line = f.readline()
        rec = cl.parse_line(first_line)
        self.assertIsNotNone(rec)
        self.assertEqual(rec.iface, "can7")
        self.assertEqual(t0, rec.t)


if __name__ == "__main__":
    unittest.main()
