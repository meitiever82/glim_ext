#!/usr/bin/env python3
"""can_sigma_patch.py —— 给 candump 日志的每个 CGI-610 周期补上全零 σ 帧(Task 3)。

背景(Task 1 实测):这台车的 610 CAN 输出没有 0x326(位置 σ)、0x328(速度 σ)、
0x32B(姿态 σ)三条报文;`gnss_chcnav_can` 驱动要求一个周期 14 条报文齐全
(`Cycle::COMPLETE_MASK = 0x3FFF`)才发布,缺了就整周期丢弃,一条消息都不发。
控制者裁定:回放时用一份补了全零 σ 帧的副本,让真实驱动能发布;σ 值是假的,
结果文档里 can 路的 σ 必须标明"合成"。

规则:
- 周期从每条 0x320 开始,到下一条 0x320 之前结束;第一条 0x320 之前的行不属于任何周期,
  原样输出、不补。
- 每个周期里缺哪几个 σ ID 就补哪几个,按 0x326、0x328、0x32B 的顺序,插在本周期最后一条
  原始帧之后(在本周期里跟在它后面的无法解析的行/空行之前)。
- 补帧的时间戳文本、接口名照抄本周期最后一条原始帧(时间戳按原文复制,不经过浮点格式化),
  数据 8 字节全零。
- 已经齐全的周期不动,所以对输出再做一次是恒等变换(幂等)。
- 最后一个周期即使不完整(真实日志末尾缺 0x32E)也照样补。

纯函数 `patch_lines` 不做 I/O;`main` 是命令行包装:`can_sigma_patch.py <in> <out>`。
"""
import re
import sys
from typing import Iterable, Iterator, List, Optional

import candump_log

CYCLE_START_ID = 0x320
SIGMA_IDS = (0x326, 0x328, 0x32B)
_ZERO_PAYLOAD = "0000000000000000"
_TS_TEXT_RE = re.compile(r"^\s*(\([0-9]+\.[0-9]+\))")


class PatchStats:
    """patch_lines 的统计:周期数(遇到的 0x320 条数)与补进去的帧数。"""

    def __init__(self) -> None:
        self.cycles = 0
        self.inserted = 0


def _flush_cycle(buf: List[str], stats: PatchStats) -> Iterator[str]:
    """输出一个周期的缓冲行,缺的 σ 帧插在最后一条可解析帧之后。"""
    last_idx = -1
    last_rec = None
    present = set()
    for i, ln in enumerate(buf):
        rec = candump_log.parse_line(ln)
        if rec is None:
            continue
        last_idx, last_rec = i, rec
        present.add(rec.can_id)
    missing = [cid for cid in SIGMA_IDS if cid not in present]
    synth: List[str] = []
    if last_rec is not None and missing:
        ts_text = _TS_TEXT_RE.match(buf[last_idx]).group(1)
        synth = ["%s %s %03X#%s\n" % (ts_text, last_rec.iface, cid, _ZERO_PAYLOAD) for cid in missing]
        stats.inserted += len(synth)
    for i, ln in enumerate(buf):
        yield ln
        if i == last_idx:
            yield from synth


def patch_lines(lines: Iterable[str], stats: Optional[PatchStats] = None) -> Iterator[str]:
    """逐行处理 candump 日志,产出补齐 σ 帧后的行(每行以单个 '\\n' 结尾)。"""
    if stats is None:
        stats = PatchStats()
    buf: List[str] = []
    in_cycle = False
    for raw in lines:
        ln = raw.rstrip("\r\n") + "\n"
        rec = candump_log.parse_line(ln)
        if rec is not None and rec.can_id == CYCLE_START_ID:
            if in_cycle:
                yield from _flush_cycle(buf, stats)
            buf = [ln]
            in_cycle = True
            stats.cycles += 1
            continue
        if in_cycle:
            buf.append(ln)
        else:
            yield ln
    if in_cycle:
        yield from _flush_cycle(buf, stats)


def main(argv: List[str]) -> int:
    if len(argv) != 3:
        print("usage: can_sigma_patch.py <in.candump.log> <out.candump.log>", file=sys.stderr)
        return 2
    stats = PatchStats()
    lines_out = 0
    with open(argv[1]) as fin, open(argv[2], "w") as fout:
        for ln in patch_lines(fin, stats):
            fout.write(ln)
            lines_out += 1
    print("can_sigma_patch: %s -> %s: cycles=%d inserted=%d lines_out=%d"
          % (argv[1], argv[2], stats.cycles, stats.inserted, lines_out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
