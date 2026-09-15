#!/usr/bin/env python3
"""tools/field_replay/field_eval.py 的单元测试(`python3 -m unittest`,全部合成数据,不读数据段)。"""
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import field_eval as fe  # noqa: E402

# GPST 周 2436 周内秒 204589.000 = 2026/09/15 08:49:49 GPST = unix(UTC) 1789462171.0
# (315964800 + 2436*604800 + 204589 - 18;inventory.md 的 t0 核对式)
T_UTC = 1789462171.0

RNX2RTKP_POS = """\
% program   : rnx2rtkp ver.EX 2.5.1
% obs start : 2026/09/15 08:49:49.0 GPST (week2436 204589.0s)
% ref pos   :  44.519819020   90.259104320   615.0870
%
% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=# of satellites)
%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
2436 204589.000   44.470303520   90.294603461   613.3524   2  21   0.9212   0.8230   2.5620   0.3087  -0.5878  -0.7088   0.00    0.0
2436 204601.000   44.470302864   90.294602471   615.8615   1  19   0.0034   0.0031   0.0099   0.0009  -0.0028  -0.0019   0.00    5.1
"""

POS_WRITER_GPST = """\
% program   : gnss_core write_pos
% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=GPST)
%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
2026/09/15 08:49:49.020   44.470257130   90.294514980   610.6390   1  39   0.0000   0.0000   0.0000   0.0000   0.0000   0.0000   0.80    0.0
2026/09/15 08:51:26.000   44.470199136   90.296070589   615.0386   4   3   4.2605   2.0374   5.0001   0.0000   0.0000   0.0000   0.00    1.6
"""

POS_UTC = """\
% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=UTC)
%  UTC                   latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
2026/09/15 08:49:31.000   44.470257130   90.294514980   610.6390   1  39   0.0000   0.0000   0.0000   0.0000   0.0000   0.0000   0.80    0.0
"""


def write_tmp(text):
    fd, path = tempfile.mkstemp(suffix=".pos")
    with os.fdopen(fd, "w") as f:
        f.write(text)
    return path


class ReadPosTest(unittest.TestCase):
    def test_rnx2rtkp_week_tow_gpst_converted_to_utc(self):
        p = write_tmp(RNX2RTKP_POS)
        try:
            pf = fe.read_pos(p)
        finally:
            os.unlink(p)
        self.assertEqual(pf.time_system, "GPST")
        self.assertEqual(len(pf.records), 2)
        r0, r1 = pf.records
        self.assertAlmostEqual(r0.t, T_UTC, places=6)
        self.assertAlmostEqual(r1.t, T_UTC + 12.0, places=6)
        self.assertAlmostEqual(r0.lat, 44.470303520, places=9)
        self.assertAlmostEqual(r0.lon, 90.294603461, places=9)
        self.assertAlmostEqual(r0.height, 613.3524, places=4)
        self.assertEqual((r0.q, r0.ns), (2, 21))
        self.assertAlmostEqual(r0.sdn, 0.9212, places=4)
        self.assertAlmostEqual(r0.sde, 0.8230, places=4)
        self.assertAlmostEqual(r0.sdu, 2.5620, places=4)
        self.assertAlmostEqual(r1.ratio, 5.1, places=4)
        self.assertEqual(r1.q, 1)

    def test_rnx2rtkp_ref_pos_header_llh(self):
        p = write_tmp(RNX2RTKP_POS)
        try:
            pf = fe.read_pos(p)
        finally:
            os.unlink(p)
        self.assertEqual(pf.ref_llh, (44.519819020, 90.259104320, 615.0870))

    def test_pos_writer_date_time_gpst_header(self):
        p = write_tmp(POS_WRITER_GPST)
        try:
            pf = fe.read_pos(p)
        finally:
            os.unlink(p)
        self.assertEqual(pf.time_system, "GPST")
        self.assertIsNone(pf.ref_llh)
        self.assertEqual(len(pf.records), 2)
        self.assertAlmostEqual(pf.records[0].t, T_UTC + 0.020, places=6)
        self.assertAlmostEqual(pf.records[1].t, T_UTC + 97.0, places=6)
        self.assertEqual(pf.records[1].q, 4)
        self.assertAlmostEqual(pf.records[1].age, 0.0)
        self.assertAlmostEqual(pf.records[1].ratio, 1.6)

    def test_utc_header_is_not_shifted(self):
        p = write_tmp(POS_UTC)
        try:
            pf = fe.read_pos(p)
        finally:
            os.unlink(p)
        self.assertEqual(pf.time_system, "UTC")
        self.assertAlmostEqual(pf.records[0].t, T_UTC, places=6)

    def test_same_instant_in_both_formats_gives_same_utc(self):
        a = fe.parse_pos_data_line("2436 204589.000 44.1 90.2 600.0 1 10 0 0 0 0 0 0 0 0", "GPST")
        b = fe.parse_pos_data_line("2026/09/15 08:49:49.000 44.1 90.2 600.0 1 10 0 0 0 0 0 0 0 0", "GPST")
        self.assertAlmostEqual(a.t, b.t, places=6)
        self.assertAlmostEqual(a.t, T_UTC, places=6)

    def test_week_rollover_tow(self):
        # 周末最后一秒与下一周第 0 秒相差 1 s
        a = fe.parse_pos_data_line("2436 604799.000 44 90 600 1 10 0 0 0", "GPST")
        b = fe.parse_pos_data_line("2437 0.000 44 90 600 1 10 0 0 0", "GPST")
        self.assertAlmostEqual(b.t - a.t, 1.0, places=6)

    def test_short_or_garbage_lines_are_rejected(self):
        self.assertIsNone(fe.parse_pos_data_line("", "GPST"))
        self.assertIsNone(fe.parse_pos_data_line("2436 204589.000 44.1 90.2", "GPST"))
        self.assertIsNone(fe.parse_pos_data_line("abc def 44 90 600 1 10 0 0 0", "GPST"))

    def test_xyz_solution_file_is_refused(self):
        text = ("%  GPST                  x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns\n"
                "2436 204589.000  -20601.3842 4555556.3139 4449890.0097 1 10 0 0 0\n")
        p = write_tmp(text)
        try:
            with self.assertRaises(ValueError):
                fe.read_pos(p)
        finally:
            os.unlink(p)

    def test_column_name_line_alone_sets_utc(self):
        # 没有 "time=" 标注、只有 RTKLIB 列名行 "%  UTC ..." 时,也要按 UTC(gnss_core read_pos 同规则)
        p = write_tmp("%  UTC                   latitude(deg) longitude(deg)  height(m)   Q  ns\n"
                      "2026/09/15 08:49:31.000 44 90 600 1 10 0 0 0\n")
        try:
            pf = fe.read_pos(p)
        finally:
            os.unlink(p)
        self.assertEqual(pf.time_system, "UTC")
        self.assertAlmostEqual(pf.records[0].t, T_UTC, places=6)

    def test_missing_time_system_defaults_to_gpst_like_gnss_core(self):
        p = write_tmp("2026/09/15 08:49:49.000 44 90 600 1 10 0 0 0\n")
        try:
            pf = fe.read_pos(p)
        finally:
            os.unlink(p)
        self.assertEqual(pf.time_system, "GPST")
        self.assertAlmostEqual(pf.records[0].t, T_UTC, places=6)


class PairByTimeTest(unittest.TestCase):
    def test_nearest_within_tolerance(self):
        pairs = fe.pair_by_time([10.0, 11.0, 12.0], [9.96, 11.02, 12.5], 0.1)
        self.assertEqual(pairs, [(0, 0), (1, 1)])

    def test_tolerance_boundary_inclusive(self):
        self.assertEqual(fe.pair_by_time([10.0], [10.1], 0.1), [(0, 0)])
        self.assertEqual(fe.pair_by_time([10.0], [10.1001], 0.1), [])

    def test_picks_closest_of_several_candidates(self):
        pairs = fe.pair_by_time([10.0], [9.95, 9.99, 10.03], 0.1)
        self.assertEqual(pairs, [(0, 1)])

    def test_one_to_one_closest_wins(self):
        # a[0] 与 a[1] 最近的都是 b[0];只保留更近的 a[1]
        pairs = fe.pair_by_time([10.00, 10.06], [10.05], 0.1)
        self.assertEqual(pairs, [(1, 0)])

    def test_one_to_one_keeps_earlier_when_it_is_closer(self):
        pairs = fe.pair_by_time([10.04, 10.08], [10.05], 0.1)
        self.assertEqual(pairs, [(0, 0)])

    def test_tolerance_boundary_exact_binary_values(self):
        # 0.25 与 0.5/0.75 都能精确表示:恰好等于容差必须算配上
        self.assertEqual(fe.pair_by_time([0.5], [0.75], 0.25), [(0, 0)])

    def test_unsorted_inputs_return_original_indices(self):
        pairs = fe.pair_by_time([12.0, 10.0], [10.01, 11.99], 0.1)
        self.assertEqual(sorted(pairs), [(0, 1), (1, 0)])

    def test_empty(self):
        self.assertEqual(fe.pair_by_time([], [1.0], 0.1), [])
        self.assertEqual(fe.pair_by_time([1.0], [], 0.1), [])


class QuantileTest(unittest.TestCase):
    def test_median_odd_even(self):
        self.assertAlmostEqual(fe.quantile([3, 1, 2], 0.5), 2.0)
        self.assertAlmostEqual(fe.quantile([4, 1, 3, 2], 0.5), 2.5)

    def test_p95_linear_interpolation(self):
        # 与 numpy.percentile 默认(linear)一致:1..100 的 95% 分位 = 95.05
        self.assertAlmostEqual(fe.quantile(list(range(1, 101)), 0.95), 95.05)

    def test_extremes_and_single(self):
        self.assertAlmostEqual(fe.quantile([5.0, 1.0], 0.0), 1.0)
        self.assertAlmostEqual(fe.quantile([5.0, 1.0], 1.0), 5.0)
        self.assertAlmostEqual(fe.quantile([7.0], 0.95), 7.0)

    def test_empty_returns_none(self):
        self.assertIsNone(fe.quantile([], 0.5))

    def test_stats_row(self):
        s = fe.stats([1.0, 2.0, 3.0, 4.0, 100.0])
        self.assertEqual(s["n"], 5)
        self.assertAlmostEqual(s["median"], 3.0)
        self.assertAlmostEqual(s["max"], 100.0)
        self.assertAlmostEqual(s["p95"], fe.quantile([1.0, 2.0, 3.0, 4.0, 100.0], 0.95))
        self.assertEqual(fe.stats([])["n"], 0)


class GeodesyTest(unittest.TestCase):
    def test_llh_to_ecef_equator(self):
        x, y, z = fe.llh_to_ecef(0.0, 0.0, 0.0)
        self.assertAlmostEqual(x, 6378137.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        self.assertAlmostEqual(z, 0.0, places=3)

    def test_llh_to_ecef_matches_1006_base(self):
        # inventory.md 4c:1006 ECEF (-20601.3842, 4555556.3139, 4449890.0097) ↔ 44.519819020 90.259104320 615.0870
        x, y, z = fe.llh_to_ecef(44.519819020, 90.259104320, 615.0870)
        self.assertAlmostEqual(x, -20601.3842, delta=0.002)
        self.assertAlmostEqual(y, 4555556.3139, delta=0.002)
        self.assertAlmostEqual(z, 4449890.0097, delta=0.002)

    def test_enu_offset_north_east_up(self):
        lat, lon, h = 44.47, 90.29, 600.0
        e, n, u = fe.enu_offset(lat, lon, h, lat + 1e-5, lon, h)
        self.assertAlmostEqual(n, 1.1119, delta=0.002)   # 1e-5° 纬度 ≈ 1.11 m
        self.assertAlmostEqual(e, 0.0, delta=1e-3)
        self.assertAlmostEqual(u, 0.0, delta=1e-3)
        e, n, u = fe.enu_offset(lat, lon, h, lat, lon + 1e-5, h + 2.0)
        self.assertAlmostEqual(e, 1.1119 * math.cos(math.radians(lat)), delta=0.003)
        self.assertAlmostEqual(u, 2.0, delta=1e-3)


class BodyFrameTest(unittest.TestCase):
    def test_heading_north(self):
        f, r = fe.body_decompose(0.0, 1.0, 0.0)   # 车头朝北,差在北 → 正前
        self.assertAlmostEqual(f, 1.0)
        self.assertAlmostEqual(r, 0.0)
        f, r = fe.body_decompose(1.0, 0.0, 0.0)   # 车头朝北,差在东 → 右
        self.assertAlmostEqual(f, 0.0)
        self.assertAlmostEqual(r, 1.0)

    def test_heading_east(self):
        f, r = fe.body_decompose(1.0, 0.0, 90.0)
        self.assertAlmostEqual(f, 1.0)
        self.assertAlmostEqual(r, 0.0)
        f, r = fe.body_decompose(0.0, -1.0, 90.0)   # 朝东时右手边是南
        self.assertAlmostEqual(f, 0.0)
        self.assertAlmostEqual(r, 1.0)

    def test_fixed_body_offset_is_recovered_at_any_heading(self):
        # 车体系固定偏移(前 -1.2 m、右 0.3 m)在各航向下旋到 ENU,再分解回来应不变
        for hdg in (0.0, 37.0, 123.0, 181.0, 270.0, 359.0):
            h = math.radians(hdg)
            fwd, right = -1.2, 0.3
            de = fwd * math.sin(h) + right * math.cos(h)
            dn = fwd * math.cos(h) - right * math.sin(h)
            f, r = fe.body_decompose(de, dn, hdg)
            self.assertAlmostEqual(f, fwd, places=9)
            self.assertAlmostEqual(r, right, places=9)

    def test_course_over_ground(self):
        self.assertAlmostEqual(fe.course_deg(0.0, 1.0), 0.0)
        self.assertAlmostEqual(fe.course_deg(1.0, 0.0), 90.0)
        self.assertAlmostEqual(fe.course_deg(0.0, -1.0), 180.0)
        self.assertAlmostEqual(fe.course_deg(-1.0, 0.0), 270.0)

    def test_mean_std(self):
        m, s = fe.mean_std([1.0, 2.0, 3.0])
        self.assertAlmostEqual(m, 2.0)
        self.assertAlmostEqual(s, 1.0)   # 样本标准差(n-1)
        self.assertEqual(fe.mean_std([]), (None, None))

    def test_linear_fit(self):
        a, b = fe.linear_fit([0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 5.0, 7.0])
        self.assertAlmostEqual(a, 1.0)
        self.assertAlmostEqual(b, 2.0)
        self.assertIsNone(fe.linear_fit([1.0, 1.0], [2.0, 3.0]))


class EventsLogTest(unittest.TestCase):
    LOG = [
        "% gnss_core events.log (time=UTC)",
        "2026/09/15 08:51:16.680 OPEN serious device_divergence lat=44.469842334 lon=90.295991901 "
        "610 输出与独立解算偏差 7.73m——疑似 610 融合问题",
        "2026/09/15 08:57:58.500 CLOSE serious device_divergence lat=44.470248087 lon=90.294546461 "
        "opened=2026/09/15 08:51:16.680 duration_s=401.8 reason=shutdown "
        "peak=corr_gap_s=4.680;divergence_m=14.200;sats_min=3.000 610 输出与独立解算偏差 7.73m——疑似 610 融合问题",
        "2026/09/15 08:57:53.680 OPEN warning no_solution lat=- lon=- 独立解算无输出——rtkrcv 未运行或未收敛",
    ]

    def test_parse_open_close(self):
        evs = fe.parse_events(self.LOG)
        self.assertEqual(len(evs), 3)
        o, c, n = evs
        self.assertEqual((o.kind, o.level, o.code), ("OPEN", "serious", "device_divergence"))
        self.assertAlmostEqual(o.t, T_UTC + 105.68, places=3)   # 08:51:16.680 UTC
        self.assertAlmostEqual(o.lat, 44.469842334)
        self.assertEqual(c.kind, "CLOSE")
        self.assertAlmostEqual(c.duration_s, 401.8)
        self.assertEqual(c.reason, "shutdown")
        self.assertAlmostEqual(c.opened_t, o.t, places=3)
        self.assertEqual(c.peak, {"corr_gap_s": 4.68, "divergence_m": 14.2, "sats_min": 3.0})
        self.assertIsNone(n.lat)
        self.assertIn("rtkrcv", n.message)

    def test_intervals_pair_open_with_close_and_leave_unclosed_open(self):
        iv = fe.event_intervals(fe.parse_events(self.LOG))
        self.assertEqual(len(iv), 2)
        dd = [x for x in iv if x.code == "device_divergence"][0]
        self.assertAlmostEqual(dd.t_close - dd.t_open, 401.82, places=2)
        ns = [x for x in iv if x.code == "no_solution"][0]
        self.assertIsNone(ns.t_close)


class FtimeTest(unittest.TestCase):
    def test_millisecond_rounding_carries_into_seconds(self):
        # x.9996 s 四舍五入到毫秒应进位成下一秒 .000,不能打印成本秒 .000
        self.assertEqual(fe.ftime(T_UTC + 0.9996), "08:49:32.000")
        self.assertEqual(fe.ftime(T_UTC + 59.9996), "08:50:31.000")

    def test_plain_and_gpst(self):
        self.assertEqual(fe.ftime(T_UTC + 0.02), "08:49:31.020")
        self.assertEqual(fe.ftime(T_UTC, gpst=True), "08:49:49.000")
        self.assertEqual(fe.ftime(None), "—")


def _offsets(headings, body=None, geo=None):
    """按航向序列造 (de, dn):body=(前, 右) 车体系固定偏移,geo=(de, dn) 地理系固定偏移。"""
    de, dn = [], []
    for i, hdg in enumerate(headings):
        noise = 0.01 * math.sin(7.0 * i)
        e = n = 0.0
        if body is not None:
            h = math.radians(hdg)
            e += body[0] * math.sin(h) + body[1] * math.cos(h)
            n += body[0] * math.cos(h) - body[1] * math.sin(h)
        if geo is not None:
            e += geo[0]
            n += geo[1]
        de.append(e + noise)
        dn.append(n - noise)
    return de, dn


class OffsetClassificationTest(unittest.TestCase):
    def test_circular_std_of_headings(self):
        self.assertAlmostEqual(fe.heading_circular_std_deg([45.0] * 10), 0.0, places=6)
        self.assertAlmostEqual(fe.heading_circular_std_deg([359.0, 1.0]), 1.0, delta=0.01)   # 跨 0° 不能当成 180° 散布
        self.assertGreater(fe.heading_circular_std_deg([0.0, 90.0, 180.0, 270.0] * 5), 90.0)
        self.assertIsNone(fe.heading_circular_std_deg([]))

    def test_straight_drive_with_geographic_offset_is_not_body_fixed(self):
        hdgs = [45.0 + 0.5 * math.sin(i) for i in range(100)]   # 直线行驶,航向几乎不变
        de, dn = _offsets(hdgs, geo=(3.0, -2.0))
        kind, _ = fe.classify_offset(de, dn, hdgs)
        self.assertNotEqual(kind, "body_fixed")
        self.assertEqual(kind, "insufficient_heading")
        # 航向完全不变时车体系与 ENU 标准差相等,旧判据(车体系标准差 ≤ ENU 标准差)会误判为杆臂
        hdgs = [45.0] * 100
        de, dn = _offsets(hdgs, geo=(3.0, -2.0))
        self.assertEqual(fe.classify_offset(de, dn, hdgs)[0], "insufficient_heading")

    def test_turning_drive_with_body_offset_is_body_fixed(self):
        hdgs = [i * 3.6 for i in range(100)]   # 转满一圈
        de, dn = _offsets(hdgs, body=(-8.45, 1.85))
        kind, text = fe.classify_offset(de, dn, hdgs)
        self.assertEqual(kind, "body_fixed")
        self.assertIn("-8.45", text)

    def test_turning_drive_with_geographic_offset_is_geo_fixed(self):
        hdgs = [i * 3.6 for i in range(100)]
        de, dn = _offsets(hdgs, geo=(3.0, -2.0))
        kind, _ = fe.classify_offset(de, dn, hdgs)
        self.assertEqual(kind, "geo_fixed")

    def test_small_offset_and_too_few_samples(self):
        hdgs = [i * 3.6 for i in range(100)]
        de, dn = _offsets(hdgs, body=(0.05, 0.02))
        self.assertEqual(fe.classify_offset(de, dn, hdgs)[0], "small")
        self.assertEqual(fe.classify_offset(de[:10], dn[:10], hdgs[:10])[0], "too_few")


class EmptyRefTest(unittest.TestCase):
    """ref 没有记录(空文件/只有头部/格式不认识)时:清楚的报错、退出码 2、不写半截报告。"""

    def test_ref_without_records_exits_2_without_traceback(self):
        import contextlib
        import io
        with tempfile.TemporaryDirectory() as d:
            ref = os.path.join(d, "rtk_check.pos")
            with open(ref, "w") as f:
                f.write("% program   : RTKLIB-EX 2.5.1\n% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float)\n")
            run = os.path.join(d, "run_X")
            # 带一条固定解的 rtkrcv.pos:修复前"首次固定相对数据起点"一行拿 None 做减法,抛 TypeError
            os.makedirs(os.path.join(run, "pos", "20260915"))
            with open(os.path.join(run, "pos", "20260915", "rtkrcv.pos"), "w") as f:
                f.write(POS_WRITER_GPST)
            out = os.path.join(d, "eval.md")
            err = io.StringIO()
            with contextlib.redirect_stderr(err):
                rc = fe.main(["--run", run, "--ref", ref, "--out", out])
            self.assertEqual(rc, 2)
            self.assertIn("没有可解析的解算记录", err.getvalue())
            self.assertIn(ref, err.getvalue())
            self.assertFalse(os.path.exists(out))


if __name__ == "__main__":
    unittest.main()
