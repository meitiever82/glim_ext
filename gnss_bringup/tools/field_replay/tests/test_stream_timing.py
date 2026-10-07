#!/usr/bin/env python3
"""tools/field_replay/stream_timing.py 的单元测试(`python3 -m unittest`;
开发机 pytest 9.1.1 与 ament launch_testing 插件冲突,不用 pytest,见
CLAUDE.md 与 MEMORY.md 的 dev-box-pytest-breaks-ros-tests 记录)。

真实数据段的用例只在 `<seg>` 存在时跑,否则 skipTest——数据集只读,不进 git。
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import stream_timing as st  # noqa: E402

SEG = "/home/steve/Documents/Datasets/tage/hongshaquan/20260915/seg_164931_165748"


class _BitWriter:
    """MSB 优先的位写入器,写完按 0 位补齐到字节边界(测试专用,不进产品代码)。"""

    def __init__(self):
        self._bits = []

    def put(self, value: int, nbits: int) -> "_BitWriter":
        for i in range(nbits - 1, -1, -1):
            self._bits.append((value >> i) & 1)
        return self

    def bytes(self) -> bytes:
        bits = list(self._bits)
        bits += [0] * ((-len(bits)) % 8)
        out = bytearray()
        for i in range(0, len(bits), 8):
            b = 0
            for bit in bits[i:i + 8]:
                b = (b << 1) | bit
            out.append(b)
        return bytes(out)


def build_rtcm3_frame(payload: bytes) -> bytes:
    """按 payload 组一条完整 RTCM3 帧(0xD3 + 10 位长度 + payload + CRC-24Q)。"""
    length = len(payload)
    assert 0 <= length < 1024
    header = bytes([0xD3, (length >> 8) & 0x03, length & 0xFF])
    crc = st.crc24q(header + payload)
    return header + payload + bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])


def build_msm_payload(msg_num: int, station_id: int, tow_field: int, tow_bits: int = 30,
                       dow_field: int = None) -> bytes:
    """组一条 MSM 头部(消息号 12 + 站号 12 + [星期 3] + 历元时间)的最小 payload,
    只覆盖 rtcm3_epoch_gpst_tow 需要读的比特,不含后续 MSM 字段(不影响分帧/CRC/时间解析)。"""
    w = _BitWriter().put(msg_num, 12).put(station_id, 12)
    if dow_field is not None:
        w.put(dow_field, 3)
    w.put(tow_field, tow_bits)
    return w.bytes()


def build_rtcm3_1005(station_id: int = 1) -> bytes:
    """一条不带时间的帧(1005 基准站坐标),仅用于 schedule() 的继承规则测试,
    内容不需要合法(rtcm3_epoch_gpst_tow 对非 MSM 类型直接返回 None)。"""
    w = _BitWriter().put(1005, 12).put(station_id, 12).put(0, 40)
    return build_rtcm3_frame(w.bytes())


def novatel_crc32(data: bytes) -> int:
    """独立于被测模块重新实现一遍 NovAtel CRC-32(poly 0xEDB88320,init 0,
    见 RTKLIB rtkcmn.c rtk_crc32),用来构造/校验测试帧,避免测试与实现共用
    同一份 CRC 代码而漏掉双方都错的情况。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc & 0xFFFFFFFF


def build_novatel_long(msg_id: int, week: int, ms: int, body: bytes = b"\x00" * 8,
                        bad_crc: bool = False) -> bytes:
    """长头(AA 44 12,头长 28):字节布局按 RTKLIB novatel.c decode_oem4/input_oem4 核实
    ——头长恒为 28(byte[3] 只是声明值,RTKLIB 解码不读它,这里如实写 28);
    msg_id@4-5(u16 LE)、消息体长度@8-9(u16 LE)、周@14-15(u16 LE)、
    毫秒@16-19(u32 LE);CRC-32 覆盖头+体,小端 4 字节紧跟其后。"""
    header = bytearray(28)
    header[0:3] = bytes([0xAA, 0x44, 0x12])
    header[3] = 28
    header[4:6] = msg_id.to_bytes(2, "little")
    header[8:10] = len(body).to_bytes(2, "little")
    header[14:16] = week.to_bytes(2, "little")
    header[16:20] = ms.to_bytes(4, "little")
    frame = bytes(header) + body
    crc = novatel_crc32(frame)
    if bad_crc:
        crc ^= 0xFFFFFFFF
    return frame + crc.to_bytes(4, "little")


def build_novatel_short(msg_id: int, week: int, ms: int, body: bytes = b"\x00" * 8) -> bytes:
    """短头(AA 44 13,头长固定 12):byte[3] = 消息体长度(u8),msg_id@4-5(u16 LE)、
    周@6-7(u16 LE)、毫秒@8-11(u32 LE)。

    注意:task-2-brief.md 原文写的是"周取 4-5 字节,毫秒取 6-9 字节",没有给出
    msg_id 的位置,且与 NovAtel OEM7 短头规范不符;已用 `<seg>/raw/cgi610.dat`
    的第一条真实帧核实——该帧 msg_id@4-5=325(RAWIMUSB)、week@6-7=2436、
    ms@8-11=204589000,三者与数据集清单(GPS 周 2436、段起始 tow 204589.0)
    完全吻合,故采用此处的偏移,brief 的短头描述记为已订正(见任务报告)。
    """
    header = bytearray(12)
    header[0:3] = bytes([0xAA, 0x44, 0x13])
    header[3] = len(body)
    header[4:6] = msg_id.to_bytes(2, "little")
    header[6:8] = week.to_bytes(2, "little")
    header[8:12] = ms.to_bytes(4, "little")
    frame = bytes(header) + body
    crc = novatel_crc32(frame)
    return frame + crc.to_bytes(4, "little")


class Crc24qTest(unittest.TestCase):
    def test_known_vector(self):
        # CRC-24Q(poly 0x1864CFB,init 0)对空串为 0;对单字节 0x00 也应为可复现的
        # 确定值——这里只验证"确定性 + 改一个 bit 必然改变结果",不依赖外部工具。
        self.assertEqual(st.crc24q(b""), 0)
        a = st.crc24q(b"\x01\x02\x03")
        b = st.crc24q(b"\x01\x02\x04")
        self.assertNotEqual(a, b)
        self.assertEqual(a & ~0xFFFFFF, 0)

    def test_ascii_check_value(self):
        # CRC-24Q(poly 0x1864CFB,init 0)对 ASCII "123456789" 的标准校验值。
        self.assertEqual(st.crc24q(b"123456789"), 0xCDE703)


class NovatelCrc32Test(unittest.TestCase):
    def test_ascii_input_differs_from_standard_crc32(self):
        # RTKLIB rtk_crc32(NovAtel 用的变体)初值 0、不做最终异或,不是标准
        # CRC-32/zlib(init=xorout=0xFFFFFFFF,标准校验值 0xCBF43926)。这里
        # 显式记录两者不同,避免以后有人假设它符合标准 CRC-32 校验值。
        self.assertNotEqual(st.novatel_crc32(b"123456789"), 0xCBF43926)

    def test_known_value_from_real_frame_bytes(self):
        # `<seg>/raw/cgi610.dat` 的第一条真实短头帧(52 字节头+体,字面量
        # 内嵌,不依赖数据集是否存在),接收机自己算出的 CRC 就是这个值,
        # RTKLIB 的变体应当与之吻合。
        frame_and_crc = bytes.fromhex(
            "aa44132845018409c8c7310c840900000000000068f908417725000085fb"
            "00002d0200007dffffffccffffff0d000000fcffffff46986236")
        frame, stored_crc = frame_and_crc[:-4], frame_and_crc[-4:]
        self.assertEqual(st.novatel_crc32(frame), int.from_bytes(stored_crc, "little"))


class Rtcm3FramingTest(unittest.TestCase):
    def test_gps_msm4_1074(self):
        payload = build_msm_payload(1074, station_id=1, tow_field=204589000)
        frame = build_rtcm3_frame(payload)
        frames = list(st.iter_rtcm3_frames(frame))
        self.assertEqual(len(frames), 1)
        f = frames[0]
        self.assertEqual(f.offset, 0)
        self.assertEqual(f.msg_type, 1074)
        self.assertTrue(f.crc_ok)
        self.assertEqual(f.raw, frame)
        self.assertEqual(st.rtcm3_epoch_gpst_tow(f), 204589.0)

    def test_garbage_before_frame_is_skipped(self):
        payload = build_msm_payload(1074, station_id=1, tow_field=204589000)
        frame = build_rtcm3_frame(payload)
        garbage = b"\x00\xff\xd3\x01"  # 含一个假的 0xD3 起始但长度不够/CRC 不过
        data = garbage + frame
        frames = list(st.iter_rtcm3_frames(data))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].offset, len(garbage))
        self.assertEqual(frames[0].msg_type, 1074)

    def test_corrupted_byte_breaks_crc_and_frame_not_produced(self):
        payload = build_msm_payload(1074, station_id=1, tow_field=204589000)
        frame = bytearray(build_rtcm3_frame(payload))
        frame[5] ^= 0xFF  # 改 payload 中间一个字节,不动前导与长度域
        frames = list(st.iter_rtcm3_frames(bytes(frame)))
        self.assertEqual(frames, [])

    def test_resync_after_corrupted_frame_finds_next_good_frame(self):
        bad = bytearray(build_rtcm3_frame(build_msm_payload(1074, 1, 204589000)))
        bad[5] ^= 0xFF
        good = build_rtcm3_frame(build_msm_payload(1074, 1, 204590000))
        frames = list(st.iter_rtcm3_frames(bytes(bad) + good))
        self.assertEqual(len(frames), 1)
        self.assertEqual(st.rtcm3_epoch_gpst_tow(frames[0]), 204590.0)

    def test_bds_msm4_1124_tow_plus_14s(self):
        payload = build_msm_payload(1124, station_id=1, tow_field=204575000)
        frame = build_rtcm3_frame(payload)
        f = list(st.iter_rtcm3_frames(frame))[0]
        self.assertEqual(f.msg_type, 1124)
        self.assertEqual(st.rtcm3_epoch_gpst_tow(f), 204589.0)

    def test_glonass_msm4_1084_dow_and_moscow_offset(self):
        # 星期 2(周二)、GPST 日内秒 08:49:49 = 31789 s;
        # 日内 ms(莫斯科时间 = GPST - 18s 闰秒 + 3h)= (31789 - 18 + 10800) * 1000
        tod_ms = (31789 - 18 + 3 * 3600) * 1000
        payload = build_msm_payload(1084, station_id=1, tow_field=tod_ms, tow_bits=27, dow_field=2)
        frame = build_rtcm3_frame(payload)
        f = list(st.iter_rtcm3_frames(frame))[0]
        self.assertEqual(f.msg_type, 1084)
        self.assertEqual(st.rtcm3_epoch_gpst_tow(f), 204589.0)

    def test_non_msm_frame_returns_none_epoch(self):
        frame = build_rtcm3_1005(station_id=7)
        f = list(st.iter_rtcm3_frames(frame))[0]
        self.assertEqual(f.msg_type, 1005)
        self.assertIsNone(st.rtcm3_epoch_gpst_tow(f))

    def test_bds_tow_wraps_into_next_week(self):
        # BDT tow = 604790 s,+14s 闰秒差 = 604804 s,超出一周(604800s),
        # 必须折回 604804 - 604800 = 4.0,而不是原样返回 604804.0。
        payload = build_msm_payload(1124, station_id=1, tow_field=604790000)
        f = list(st.iter_rtcm3_frames(build_rtcm3_frame(payload)))[0]
        self.assertEqual(st.rtcm3_epoch_gpst_tow(f), 4.0)

    def test_glonass_wraps_into_previous_week_near_sunday_midnight(self):
        # 星期 0(周日)、莫斯科时间 01:00(尚未到 GPST 周起点):
        # GPST 日内秒 = 3600 - 3h + 18s = -7182,总数 -7182 落在上一周,
        # 折回 604800 - 7182 = 597618.0(而不是原样返回负数)。
        tod_ms = 3600 * 1000
        payload = build_msm_payload(1084, station_id=1, tow_field=tod_ms, tow_bits=27, dow_field=0)
        f = list(st.iter_rtcm3_frames(build_rtcm3_frame(payload)))[0]
        self.assertEqual(st.rtcm3_epoch_gpst_tow(f), 597618.0)

    def test_glonass_dow_unknown_returns_none(self):
        # DF416:星期字段值 7 表示"未知",没有星期就没法算出周内秒,
        # 只能返回 None,交给 schedule() 的继承规则处理。
        payload = build_msm_payload(1084, station_id=1, tow_field=0, tow_bits=27, dow_field=7)
        f = list(st.iter_rtcm3_frames(build_rtcm3_frame(payload)))[0]
        self.assertIsNone(st.rtcm3_epoch_gpst_tow(f))


class NovatelFramingTest(unittest.TestCase):
    def test_long_header_inspvaxb(self):
        raw = build_novatel_long(msg_id=1465, week=2436, ms=204589000)
        frames = list(st.iter_novatel_frames(raw))
        self.assertEqual(len(frames), 1)
        f = frames[0]
        self.assertEqual(f.msg_id, 1465)
        self.assertEqual(f.week, 2436)
        self.assertEqual(f.ms, 204589000)
        self.assertTrue(f.crc_ok)
        self.assertEqual(f.offset, 0)

    def test_long_header_bad_crc_still_yielded_but_flagged(self):
        raw = build_novatel_long(msg_id=140, week=2436, ms=1000, bad_crc=True)
        f = list(st.iter_novatel_frames(raw))[0]
        self.assertFalse(f.crc_ok)
        self.assertEqual(f.msg_id, 140)

    def test_short_header_rawimusb(self):
        raw = build_novatel_short(msg_id=325, week=2436, ms=204589000)
        frames = list(st.iter_novatel_frames(raw))
        self.assertEqual(len(frames), 1)
        f = frames[0]
        self.assertEqual(f.msg_id, 325)
        self.assertEqual(f.week, 2436)
        self.assertEqual(f.ms, 204589000)
        self.assertTrue(f.crc_ok)

    def test_mixed_stream_long_then_short(self):
        long_f = build_novatel_long(msg_id=1465, week=2436, ms=204589000)
        short_f = build_novatel_short(msg_id=325, week=2436, ms=204589100)
        frames = list(st.iter_novatel_frames(long_f + short_f))
        self.assertEqual([f.msg_id for f in frames], [1465, 325])
        self.assertEqual(frames[1].offset, len(long_f))

    def test_false_sync_with_oversized_length_does_not_skip_next_valid_frame(self):
        # 一个"假同步"(AA 44 12 出现在别处,声明的长度域是随机数据)如果
        # CRC 不过时仍按声明的 total_len 前跳,会把紧跟其后的真帧整个跳过去。
        # 这里把一条完整合法帧紧接在假头部后面,声明的消息体长度刚好覆盖
        # 假头部+这条真帧+尾部填充,凑成一个内容对不上、CRC 必然不过的
        # "巨大"候选帧;修复后应按 1 字节前跳重新同步,找到内嵌的真帧。
        valid = build_novatel_long(msg_id=140, week=2436, ms=1000)
        header = bytearray(28)
        header[0:3] = bytes([0xAA, 0x44, 0x12])
        header[3] = 28
        declared_body_len = 200
        header[8:10] = declared_body_len.to_bytes(2, "little")
        header[14:16] = (2436).to_bytes(2, "little")
        header[16:20] = (999).to_bytes(4, "little")
        total_len = 28 + declared_body_len + 4
        body_and_pad = valid + b"\x00" * (declared_body_len - len(valid))
        fake_crc = b"\x00\x00\x00\x00"
        data = bytes(header) + body_and_pad + fake_crc
        self.assertEqual(len(data), total_len)

        frames = list(st.iter_novatel_frames(data))
        self.assertFalse(frames[0].crc_ok)  # 假帧本身 CRC 必然不过
        self.assertIn(140, [f.msg_id for f in frames])
        found = [f for f in frames if f.msg_id == 140]
        self.assertTrue(found[0].crc_ok)
        self.assertEqual(found[0].offset, 28)


class GpstToUnixUtcTest(unittest.TestCase):
    def test_known_value(self):
        # week=2436, tow=204589.0 -> 2026-09-15 08:49:31 UTC = unix 1789462171
        self.assertEqual(st.gpst_to_unix_utc(2436, 204589.0), 1789462171.0)

    def test_default_leap_seconds_is_18(self):
        self.assertEqual(st.gpst_to_unix_utc(2436, 204589.0, leap_s=18),
                          st.gpst_to_unix_utc(2436, 204589.0))


class ScheduleTest(unittest.TestCase):
    def test_leading_untimed_frames_inherit_first_time(self):
        frames = [(None, b"a"), (None, b"b"), (10.0, b"c"), (None, b"d")]
        out = st.schedule(frames)
        self.assertEqual(out, [(10.0, b"a"), (10.0, b"b"), (10.0, b"c"), (10.0, b"d")])

    def test_untimed_frame_inherits_previous_timed_frame(self):
        frames = [(1.0, b"a"), (None, b"b"), (2.0, b"c"), (None, b"d"), (None, b"e")]
        out = st.schedule(frames)
        self.assertEqual(out, [(1.0, b"a"), (1.0, b"b"), (2.0, b"c"), (2.0, b"d"), (2.0, b"e")])

    def test_no_timed_frame_raises(self):
        with self.assertRaises(ValueError):
            st.schedule([(None, b"a")])


@unittest.skipUnless(os.path.isfile(os.path.join(SEG, "gnss", "base.rtcm3")),
                      f"数据段不存在,跳过真实文件用例:{SEG}")
class RealBaseRtcm3Test(unittest.TestCase):
    def test_message_type_counts_and_crc(self):
        with open(os.path.join(SEG, "gnss", "base.rtcm3"), "rb") as f:
            data = f.read()
        frames = list(st.iter_rtcm3_frames(data))
        # iter_rtcm3_frames 对 CRC 不过的候选帧从不产出,所以
        # `all(f.crc_ok for f in frames)` 永远为真(空集合也满足),
        # 是个自我验证式的空判断;改成检查产出的帧首尾相接、无缝覆盖整个
        # 文件字节数——这才是"没有漏检/误判"的有意义证据(该文件里确认
        # 帧与帧之间没有间隙,`sum(len(raw))` 应精确等于文件长度)。
        self.assertEqual(sum(len(f.raw) for f in frames), len(data))
        counts = {}
        for f in frames:
            counts[f.msg_type] = counts.get(f.msg_type, 0) + 1
        self.assertEqual(counts.get(1006), 50)
        self.assertEqual(counts.get(1074), 498)


@unittest.skipUnless(os.path.isfile(os.path.join(SEG, "raw", "cgi610.dat")),
                      f"数据段不存在,跳过真实文件用例:{SEG}")
class RealCgi610Test(unittest.TestCase):
    def test_message_id_counts_and_crc(self):
        with open(os.path.join(SEG, "raw", "cgi610.dat"), "rb") as f:
            data = f.read()
        frames = list(st.iter_novatel_frames(data))
        self.assertTrue(all(f.crc_ok for f in frames), "cgi610.dat 中存在 CRC 不过的帧")
        counts = {}
        for f in frames:
            counts[f.msg_id] = counts.get(f.msg_id, 0) + 1
        self.assertEqual(counts.get(140), 498)     # RANGECMPB, 1 Hz
        self.assertEqual(counts.get(1465), 4974)   # INSPVAXB, 10 Hz


if __name__ == "__main__":
    unittest.main()
