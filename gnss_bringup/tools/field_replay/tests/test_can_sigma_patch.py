#!/usr/bin/env python3
"""tools/field_replay/can_sigma_patch.py 的单元测试(`python3 -m unittest`)。

背景:本车 610 的 CAN 输出里没有 0x326/0x328/0x32B 三条 σ 报文,驱动要求 14 条齐全
才发布(Task 1)。回放用的 CAN 日志要在每个周期末尾补三条全零 σ 帧。
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import can_sigma_patch as csp  # noqa: E402

ZERO = "0000000000000000"


def cycle(t_base_us, ids=(0x320, 0x321, 0x325, 0x327, 0x32E), iface="can7", t_int=1789462171):
    """造一个周期:ID 依次间隔 1 µs,时间戳写成 candump 的 6 位小数。"""
    out = []
    for k, can_id in enumerate(ids):
        out.append("(%d.%06d) %s %03X#1122334455667788" % (t_int, t_base_us + k, iface, can_id))
    return out


def ids_of(lines):
    ids = []
    for ln in lines:
        rec = csp.candump_log.parse_line(ln)
        ids.append(None if rec is None else rec.can_id)
    return ids


class PatchLinesTest(unittest.TestCase):
    def test_inserts_three_sigma_frames_after_last_frame_of_each_cycle(self):
        src = cycle(2222) + cycle(14837)
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src)]
        self.assertEqual(ids_of(out), [
            0x320, 0x321, 0x325, 0x327, 0x32E, 0x326, 0x328, 0x32B,
            0x320, 0x321, 0x325, 0x327, 0x32E, 0x326, 0x328, 0x32B,
        ])

    def test_synthesized_frames_copy_timestamp_text_and_iface_of_last_frame(self):
        src = cycle(2222, iface="can7")
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src)]
        last = src[-1]
        ts_text = last[:last.index(")") + 1]   # "(1789462171.002226)"
        self.assertEqual(out[5], ts_text + " can7 326#" + ZERO)
        self.assertEqual(out[6], ts_text + " can7 328#" + ZERO)
        self.assertEqual(out[7], ts_text + " can7 32B#" + ZERO)

    def test_original_lines_kept_verbatim_and_in_order(self):
        src = cycle(2222) + cycle(14837)
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src)]
        synth = {0x326, 0x328, 0x32B}
        kept = [ln for ln, i in zip(out, ids_of(out)) if i not in synth]
        self.assertEqual(kept, src)

    def test_every_output_line_ends_with_newline(self):
        src = [ln + "\n" for ln in cycle(2222)]
        out = list(csp.patch_lines(src))
        self.assertTrue(all(ln.endswith("\n") and not ln.endswith("\n\n") for ln in out))
        self.assertEqual(len(out), 8)

    def test_idempotent_when_frames_already_present(self):
        src = cycle(2222) + cycle(14837)
        once = list(csp.patch_lines(src))
        twice = list(csp.patch_lines(once))
        self.assertEqual(once, twice)

    def test_only_missing_sigma_ids_are_inserted(self):
        # 周期里已有 0x328:只补 0x326 与 0x32B,已有的那条原样保留在原位置
        src = cycle(2222, ids=(0x320, 0x321, 0x328, 0x32E))
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src)]
        self.assertEqual(ids_of(out), [0x320, 0x321, 0x328, 0x32E, 0x326, 0x32B])
        self.assertEqual(out[:4], src)

    def test_last_partial_cycle_is_patched_too(self):
        # 真实日志末尾的周期缺 0x32E;补 σ 仍按"本周期最后一条原始帧"之后插入
        src = cycle(2222) + cycle(14837, ids=(0x320, 0x321, 0x325))
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src)]
        self.assertEqual(ids_of(out)[8:], [0x320, 0x321, 0x325, 0x326, 0x328, 0x32B])
        ts_text = src[-1][:src[-1].index(")") + 1]
        self.assertTrue(out[-1].startswith(ts_text + " "))

    def test_lines_before_first_cycle_start_are_not_a_cycle(self):
        # 日志从周期中间开始:第一个 0x320 之前的帧不属于任何完整周期,不补
        src = ["(1789462171.000001) can7 32D#00", "(1789462171.000002) can7 32E#00"] + cycle(2222)
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src)]
        self.assertEqual(ids_of(out), [0x32D, 0x32E, 0x320, 0x321, 0x325, 0x327, 0x32E,
                                       0x326, 0x328, 0x32B])

    def test_unparseable_lines_pass_through_and_do_not_move_insertion_point(self):
        src = cycle(2222)
        src_with_junk = src[:2] + ["garbage line"] + src[2:] + [""]
        out = [ln.rstrip("\n") for ln in csp.patch_lines(src_with_junk)]
        # 插入点紧跟最后一条原始帧,其后的空行保持在插入帧之后
        self.assertEqual(out[:6], src[:2] + ["garbage line"] + src[2:])
        self.assertEqual(ids_of(out[6:9]), [0x326, 0x328, 0x32B])
        self.assertEqual(out[9], "")

    def test_empty_input(self):
        self.assertEqual(list(csp.patch_lines([])), [])

    def test_stats_counts_cycles_and_insertions(self):
        src = cycle(2222) + cycle(14837, ids=(0x320, 0x326, 0x32E))
        stats = csp.PatchStats()
        list(csp.patch_lines(src, stats))
        self.assertEqual(stats.cycles, 2)
        self.assertEqual(stats.inserted, 3 + 2)


if __name__ == "__main__":
    unittest.main()
