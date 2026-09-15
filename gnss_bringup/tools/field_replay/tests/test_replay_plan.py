#!/usr/bin/env python3
"""tools/field_replay/replay_plan.py 的单元测试(`python3 -m unittest`)。

回放按文件原始字节、按文件顺序发送:每块 = 上一帧末尾之后到本帧末尾的全部字节,
发送时刻 = 截至本帧的帧时间最大值(RANGECMPB 在文件里比 IMU 晚到,时间往回走)。
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import replay_plan as rp  # noqa: E402
import stream_timing as st  # noqa: E402
from test_stream_timing import (  # noqa: E402  帧构造器复用 Task 2 的测试夹具
    build_msm_payload, build_novatel_long, build_novatel_short, build_rtcm3_1005,
    build_rtcm3_frame)

WEEK = 2436
TOW0_MS = 204589000
T0_UNIX = 1789462171.0   # = gpst_to_unix_utc(2436, 204589.0)


def span(start, end, t):
    return rp.FrameSpan(start=start, end=end, t=t)


class ChunkStreamTest(unittest.TestCase):
    def test_bytes_between_frames_go_with_following_frame_and_nothing_is_lost(self):
        data = b"ab" + b"FRAME1" + b"\n" + b"FRAME2" + b"xyz"
        spans = [span(2, 8, 10.0), span(9, 15, 11.0)]
        chunks = rp.chunk_stream(data, spans)
        self.assertEqual([c.data for c in chunks], [b"abFRAME1", b"\nFRAME2xyz"])
        self.assertEqual(b"".join(c.data for c in chunks), data)
        self.assertEqual([c.frames for c in chunks], [1, 1])
        self.assertEqual([c.kind for c in chunks], ["data", "data"])

    def test_time_is_running_max_when_frame_times_go_backwards(self):
        data = bytes(40)
        spans = [span(0, 10, 100.18), span(10, 20, 100.0), span(20, 30, 100.19), span(30, 40, 100.1)]
        chunks = rp.chunk_stream(data, spans)
        self.assertEqual([c.t for c in chunks], [100.18, 100.18, 100.19, 100.19])

    def test_untimed_frames_inherit_previous_and_leading_ones_inherit_first(self):
        data = bytes(30)
        spans = [span(0, 10, None), span(10, 20, 5.0), span(20, 30, None)]
        chunks = rp.chunk_stream(data, spans)
        self.assertEqual([c.t for c in chunks], [5.0, 5.0, 5.0])

    def test_no_spans_or_no_times_raise(self):
        with self.assertRaises(ValueError):
            rp.chunk_stream(b"abc", [])
        with self.assertRaises(ValueError):
            rp.chunk_stream(bytes(10), [span(0, 10, None)])

    def test_overlapping_or_out_of_range_spans_raise(self):
        with self.assertRaises(ValueError):
            rp.chunk_stream(bytes(20), [span(0, 10, 1.0), span(5, 15, 2.0)])
        with self.assertRaises(ValueError):
            rp.chunk_stream(bytes(10), [span(0, 11, 1.0)])


class GpstTowNearTest(unittest.TestCase):
    def test_same_week(self):
        self.assertAlmostEqual(rp.gpst_tow_to_unix_near(204590.0, T0_UNIX), T0_UNIX + 1.0)

    def test_picks_adjacent_week_closest_to_reference(self):
        ref = st.gpst_to_unix_utc(WEEK, 604790.0)
        # 下一周刚开始的历元(tow=5)离参考时刻只有 15 s,不是 604785 s 之前
        self.assertAlmostEqual(rp.gpst_tow_to_unix_near(5.0, ref), st.gpst_to_unix_utc(WEEK + 1, 5.0))
        ref2 = st.gpst_to_unix_utc(WEEK, 3.0)
        self.assertAlmostEqual(rp.gpst_tow_to_unix_near(604799.0, ref2),
                               st.gpst_to_unix_utc(WEEK - 1, 604799.0))


class Rtcm3SpansTest(unittest.TestCase):
    def test_msm_times_and_untimed_1005(self):
        f1 = build_rtcm3_frame(build_msm_payload(1074, 1, TOW0_MS))
        f2 = build_rtcm3_1005()
        f3 = build_rtcm3_frame(build_msm_payload(1074, 1, TOW0_MS + 1000))
        data = f1 + f2 + f3
        spans = rp.rtcm3_spans(data, ref_unix=T0_UNIX)
        self.assertEqual([(s.start, s.end) for s in spans],
                         [(0, len(f1)), (len(f1), len(f1) + len(f2)), (len(f1) + len(f2), len(data))])
        self.assertEqual([s.t for s in spans], [T0_UNIX, None, T0_UNIX + 1.0])

    def test_leap_seconds_argument_used(self):
        data = build_rtcm3_frame(build_msm_payload(1074, 1, TOW0_MS))
        spans = rp.rtcm3_spans(data, ref_unix=T0_UNIX, leap_s=17)
        self.assertAlmostEqual(spans[0].t, T0_UNIX + 1.0)


class NovatelSpansTest(unittest.TestCase):
    def test_long_and_short_frames_with_junk_between(self):
        a = build_novatel_short(325, WEEK, TOW0_MS)
        b = build_novatel_long(1465, WEEK, TOW0_MS + 100)
        rtcm = build_rtcm3_frame(build_msm_payload(1074, 1, TOW0_MS))
        data = a + b"\n" + rtcm + b + b"\n"
        spans = rp.novatel_spans(data)
        self.assertEqual(len(spans), 2)
        self.assertEqual((spans[0].start, spans[0].end), (0, len(a)))
        self.assertEqual(spans[1].end, len(data) - 1)
        self.assertAlmostEqual(spans[0].t, T0_UNIX)
        self.assertAlmostEqual(spans[1].t, T0_UNIX + 0.1)
        chunks = rp.chunk_stream(data, spans)
        self.assertEqual(b"".join(c.data for c in chunks), data)
        self.assertEqual(chunks[1].data, b"\n" + rtcm + b + b"\n")

    def test_bad_crc_frames_are_not_boundaries(self):
        good = build_novatel_short(325, WEEK, TOW0_MS)
        bad = build_novatel_long(1465, WEEK, TOW0_MS + 100, bad_crc=True)
        data = good + bad
        spans = rp.novatel_spans(data)
        self.assertEqual(len(spans), 1)
        self.assertEqual(spans[0].end, len(good))


class InjectPeriodicTest(unittest.TestCase):
    def mk(self, times):
        return [rp.Chunk(t=t, data=b"d%d" % i, frames=1) for i, t in enumerate(times)]

    def test_payload_before_first_chunk_then_every_period(self):
        chunks = self.mk([100.0, 110.0, 129.0, 130.0, 145.0, 161.0])
        out = rp.inject_periodic(chunks, b"NAV", period_s=30.0, frames=7)
        kinds = [(c.kind, c.t) for c in out]
        self.assertEqual(kinds, [
            ("nav", 100.0), ("data", 100.0), ("data", 110.0), ("data", 129.0),
            ("nav", 130.0), ("data", 130.0), ("data", 145.0),
            ("nav", 160.0), ("data", 161.0),
        ])
        self.assertTrue(all(c.data == b"NAV" and c.frames == 7 for c in out if c.kind == "nav"))
        # 原数据块一个不少、顺序不变
        self.assertEqual([c.data for c in out if c.kind == "data"], [c.data for c in chunks])

    def test_gap_longer_than_period_gets_each_injection(self):
        chunks = self.mk([0.0, 70.0])
        out = rp.inject_periodic(chunks, b"N", period_s=30.0, frames=1)
        self.assertEqual([(c.kind, c.t) for c in out],
                         [("nav", 0.0), ("data", 0.0), ("nav", 30.0), ("nav", 60.0), ("data", 70.0)])

    def test_no_injection_after_last_chunk_and_empty_input(self):
        out = rp.inject_periodic(self.mk([0.0, 10.0]), b"N", period_s=30.0, frames=1)
        self.assertEqual([c.kind for c in out], ["nav", "data", "data"])
        self.assertEqual(rp.inject_periodic([], b"N", period_s=30.0, frames=1), [])

    def test_non_positive_period_raises(self):
        with self.assertRaises(ValueError):
            rp.inject_periodic(self.mk([0.0]), b"N", period_s=0.0, frames=1)


class TruncateTest(unittest.TestCase):
    def test_keeps_chunks_strictly_before_end(self):
        chunks = [rp.Chunk(t=t, data=b"x", frames=1) for t in (0.0, 5.0, 9.99, 10.0, 11.0)]
        self.assertEqual([c.t for c in rp.truncate(chunks, 10.0)], [0.0, 5.0, 9.99])
        self.assertEqual(len(rp.truncate(chunks, None)), 5)


class WallDueTest(unittest.TestCase):
    def test_speed_scales_data_offset(self):
        self.assertAlmostEqual(rp.wall_due(t=110.0, t0=100.0, w0=5.0, speed=1.0), 15.0)
        self.assertAlmostEqual(rp.wall_due(t=110.0, t0=100.0, w0=5.0, speed=10.0), 6.0)

    def test_clock_time_inverse(self):
        self.assertAlmostEqual(rp.data_time(wall=6.0, t0=100.0, w0=5.0, speed=10.0), 110.0)


PROC_NET_TCP = """  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode
   0: 0100007F:3AC1 00000000:0000 0A 00000000:00000000 00:00000000 00000000  1000        0 1 1 0 100 0 0 10 0
   1: 0100007F:3AC1 0100007F:9C40 01 00000000:00000000 00:00000000 00000000  1000        0 2 1 0 100 0 0 10 0
   2: 0100007F:3AC2 0100007F:9C41 01 00000000:00000000 00:00000000 00000000  1000        0 3 1 0 100 0 0 10 0
   3: 0100007F:9C40 0100007F:3AC1 01 00000000:00000000 00:00000000 00000000  1000        0 4 1 0 100 0 0 10 0
   4: 0100007F:3AC2 0100007F:9C42 08 00000000:00000000 00:00000000 00000000  1000        0 5 1 0 100 0 0 10 0
"""


class EstablishedPortsTest(unittest.TestCase):
    def test_counts_established_by_local_port_only(self):
        # 0x3AC1 = 15041(LISTEN 不算,ESTABLISHED 算);0x3AC2 = 15042(CLOSE_WAIT=08 不算);
        # 0x9C40 是客户端一侧的本地端口,也计入(按本地端口统计,调用方只查自己关心的端口)
        counts = rp.established_local_ports(PROC_NET_TCP)
        self.assertEqual(counts.get(15041), 1)
        self.assertEqual(counts.get(15042), 1)
        self.assertEqual(counts.get(0x9C40), 1)
        self.assertNotIn(0x9C42, counts)

    def test_empty_or_header_only(self):
        self.assertEqual(rp.established_local_ports(""), {})
        self.assertEqual(rp.established_local_ports(PROC_NET_TCP.splitlines()[0]), {})


class PortsAllConnectedTest(unittest.TestCase):
    # Task 6 F2 之后 rtkrcv conf 写 misc-timeout=0:tcpcli 连上就不会因空闲断开,
    # 已经连着的连接直接可用,不再需要等"先断后连"(旧 FreshConnectionWaiter 在新 conf 下永远等不到)
    def test_already_connected_at_start_is_ready(self):
        self.assertTrue(rp.ports_all_connected([15041, 15042], {15041: 1, 15042: 1}))

    def test_waits_until_every_port_has_a_connection(self):
        self.assertFalse(rp.ports_all_connected([15041, 15042], {}))
        self.assertFalse(rp.ports_all_connected([15041, 15042], {15041: 1}))
        self.assertTrue(rp.ports_all_connected([15041, 15042], {15041: 1, 15042: 2, 9999: 1}))

    def test_zero_count_is_not_connected(self):
        self.assertFalse(rp.ports_all_connected([15041], {15041: 0}))

    def test_no_ports_is_immediately_ready(self):
        self.assertTrue(rp.ports_all_connected([], {}))


if __name__ == "__main__":
    unittest.main()
