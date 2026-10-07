#!/usr/bin/env python3
"""pos_to_rtkfix_bag.py 的纯解析函数测试(不需要 ROS;轮 5 Task 3)。

本文件被 gnss_core/CMakeLists.txt 的 `unittest discover -s tests` 自动收集。
用 unittest 而非 pytest:见该 CMakeLists 里的注释(新版 pytest 与 ROS Humble 的
launch_testing 插件不兼容)。
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

import pos_to_rtkfix_bag as mod  # noqa: E402


def write(tmpdir, name, text):
    path = os.path.join(tmpdir, name)
    with open(path, "w") as f:
        f.write(text)
    return path


class ParsePosCalendar(unittest.TestCase):
    HEADER_GPST = (
        "% program   : gnss_core write_pos\n"
        "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=GPST)\n"
        "%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns"
        "   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )
    ROW = ("2026/09/15 08:49:50.000   44.470305335   90.294609181   610.9932   2  25"
           "   0.8573   0.7637   1.9293   0.0000   0.0000   0.0000   1.50    0.0\n")

    def test_gpst_header_subtracts_leap_seconds(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.pos", self.HEADER_GPST + self.ROW)
            recs = mod.parse_pos(p)
            self.assertEqual(len(recs), 1)
            # 2026/09/15 08:49:50 UTC = 1789462190;GPST 头 → 减 18 s
            self.assertAlmostEqual(recs[0]["t"], 1789462190.0 - 18.0, places=3)
            self.assertEqual(recs[0]["q"], 2)
            self.assertEqual(recs[0]["ns"], 25)
            self.assertAlmostEqual(recs[0]["sdn"], 0.8573, places=6)
            self.assertAlmostEqual(recs[0]["sde"], 0.7637, places=6)
            self.assertAlmostEqual(recs[0]["age"], 1.50, places=3)

    def test_utc_header_keeps_stamp(self):
        with tempfile.TemporaryDirectory() as d:
            header = self.HEADER_GPST.replace("time=GPST", "time=UTC")
            p = write(d, "a.pos", header + self.ROW)
            recs = mod.parse_pos(p)
            self.assertAlmostEqual(recs[0]["t"], 1789462190.0, places=3)


class ParsePosWeekTow(unittest.TestCase):
    """rnx2rtkp 默认输出的 "GPS 周 + 周内秒" 时间列(gnss/rtk_check.pos 就是这个格式)。

    gnss_core::read_pos 已支持(PR #8 8fefcdc),Python 侧此前只认日历格式,
    对 rtk_check.pos 会静默解析出 0 条。
    """

    HEADER = (
        "% program   : rnx2rtkp ver.EX 2.5.1\n"
        "% obs start : 2026/09/15 08:49:49.0 GPST\n"
        "%  GPST      latitude(deg) longitude(deg)  height(m)   Q  ns"
        "   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )
    # GPS 周 2436、周内秒 204589.000 → GPST 2026/09/15 08:49:49 → UTC 08:49:31
    ROW = ("2436 204589.000   44.470303520   90.294603461   613.3524   2  22"
           "   0.0123   0.0098   0.0345   0.0000   0.0000   0.0000   0.90    1.2\n")

    def test_week_tow_is_parsed_as_gpst(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.pos", self.HEADER + self.ROW)
            recs = mod.parse_pos(p)
            self.assertEqual(len(recs), 1)
            # GPS epoch 1980-01-06T00:00:00Z = 315964800
            expected_gpst = 315964800.0 + 2436 * 604800.0 + 204589.000
            self.assertAlmostEqual(recs[0]["t"], expected_gpst - 18.0, places=3)

    def test_week_tow_honours_utc_header(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.pos", self.HEADER.replace("GPST", "UTC") + self.ROW)
            recs = mod.parse_pos(p)
            expected = 315964800.0 + 2436 * 604800.0 + 204589.000
            self.assertAlmostEqual(recs[0]["t"], expected, places=3)

    def test_week_tow_rejects_out_of_range_tow(self):
        with tempfile.TemporaryDirectory() as d:
            bad = self.ROW.replace("204589.000", "999999.000")   # > 604800
            p = write(d, "a.pos", self.HEADER + bad)
            self.assertEqual(mod.parse_pos(p), [])

    def test_real_rtk_check_pos_is_not_silently_empty(self):
        """真实文件存在时顺带跑一次;不存在就跳过(测试不依赖数据集)。"""
        path = ("/home/steve/Documents/Datasets/tage/hongshaquan/20260915/"
                "seg_164931_165748/gnss/rtk_check.pos")
        if not os.path.exists(path):
            self.skipTest("dataset not present")
        recs = mod.parse_pos(path)
        self.assertEqual(len(recs), 498)


class ParseEnu(unittest.TestCase):
    TEXT = (
        "# origin_lla 44.47025713 90.29451498 610.640\n"
        "# t e n u sde sdn sdu q\n"
        "1789462172.000 7.412 -3.118 0.353 0.0764 0.0857 0.1929 4\n"
        "garbage\n"
        "1789462173.000 8.0 -3.0\n"
        "1789462174.000 9.100 -2.900 0.400 0.0800 0.0900 0.2000 3\n"
    )

    def test_parses_origin_and_skips_bad_lines(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.enu", self.TEXT)
            origin, samples = mod.parse_enu(p)
            self.assertAlmostEqual(origin[0], 44.47025713, places=8)
            self.assertAlmostEqual(origin[2], 610.640, places=3)
            self.assertEqual(len(samples), 2)
            self.assertAlmostEqual(samples[0]["e"], 7.412, places=3)
            self.assertAlmostEqual(samples[0]["sdu"], 0.1929, places=4)
            self.assertEqual(samples[0]["q"], 4)
            self.assertEqual(samples[1]["q"], 3)

    def test_crlf_lines(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.enu", self.TEXT.replace("\n", "\r\n"))
            _, samples = mod.parse_enu(p)
            self.assertEqual(len(samples), 2)

    def test_missing_origin_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(d, "a.enu", "1789462172.000 1 2 3 0.1 0.1 0.1 4\n")
            origin, samples = mod.parse_enu(p)
            self.assertIsNone(origin)
            self.assertEqual(len(samples), 1)


if __name__ == "__main__":
    unittest.main()
