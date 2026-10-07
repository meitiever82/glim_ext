#!/usr/bin/env python3
"""replay_plan.py —— 回放发送计划的纯函数(Task 3):把整文件原始字节切成按帧边界对齐、
带数据时刻的发送块,以及星历周期注入、按时长截断、数据时刻与墙钟的换算。

不做 I/O、不依赖 ROS;`field_replay.py` 用它生成两路 TCP 的发送表。

控制者裁定(2026-09-16):
- **原始字节、文件顺序**:`cgi610.dat` 里有约 430 kB 不属于 NovAtel 帧的字节(每帧后的
  `\\n`、夹带的 RTCM3 帧),必须原样全部转发。所以不是"按帧重新拼",而是把文件在帧边界处
  切开:每块 = 上一帧末尾之后到本帧末尾(含)的全部字节;第一块含文件开头的前导字节,最后
  一块带上最后一帧之后的尾随字节。所有块拼起来与原文件逐字节相同。
- **发送时刻 = 截至本帧的帧时间最大值**:RANGECMPB 在文件里排在更晚的 IMU 报文之后,时间
  最多往回走 190 ms;按文件顺序发、时间取累计最大值,不重排。
- 不带时间的帧(RTCM 1006 等)沿用前一帧的时间;开头的无时间帧沿用第一个有时间的帧
  (与 `stream_timing.schedule` 同一规则)。
"""
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence

import stream_timing as st

_GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06 00:00:00 UTC


@dataclass(frozen=True)
class FrameSpan:
    """一条帧在文件里的字节范围 [start, end) 与数据时刻(unix UTC 秒,无时间为 None)。"""
    start: int
    end: int
    t: Optional[float]


@dataclass
class Chunk:
    """一个发送块:到时刻 t(unix UTC 秒)时整块写进套接字。"""
    t: float
    data: bytes
    frames: int        # 本块包含的完整帧数(数据块为 1,星历块为电文条数)
    kind: str = "data"  # "data" = 文件原始字节;"nav" = 注入的星历电文


def chunk_stream(data: bytes, spans: Sequence[FrameSpan]) -> List[Chunk]:
    """按帧边界把整个 data 切成发送块(见模块说明);spans 必须按文件顺序、互不重叠。"""
    if not spans:
        raise ValueError("没有任何帧,无法切分发送块")
    times = st.schedule((s.t, b"") for s in spans)  # 无时间帧的继承规则;全无时间时抛 ValueError
    chunks: List[Chunk] = []
    prev_end = 0
    t_max = None
    for s, (t, _) in zip(spans, times):
        if s.start < prev_end or s.end <= s.start or s.end > len(data):
            raise ValueError("帧范围非法或重叠: [%d, %d) (上一帧结束于 %d, 文件长 %d)"
                             % (s.start, s.end, prev_end, len(data)))
        t_max = t if t_max is None else max(t_max, t)
        chunks.append(Chunk(t=t_max, data=data[prev_end:s.end], frames=1))
        prev_end = s.end
    if prev_end < len(data):
        last = chunks[-1]
        chunks[-1] = Chunk(t=last.t, data=last.data + data[prev_end:], frames=last.frames, kind=last.kind)
    return chunks


def gpst_tow_to_unix_near(tow_s: float, ref_unix: float, leap_s: int = 18) -> float:
    """RTCM MSM 只带周内秒:在参考时刻所在周及前后各一周里,取离 ref_unix 最近的那个。"""
    ref_week = int((ref_unix + leap_s - _GPS_EPOCH_UNIX) // 604800)
    candidates = [st.gpst_to_unix_utc(w, tow_s, leap_s) for w in (ref_week - 1, ref_week, ref_week + 1)]
    return min(candidates, key=lambda u: abs(u - ref_unix))


def rtcm3_spans(data: bytes, ref_unix: float, leap_s: int = 18) -> List[FrameSpan]:
    """RTCM3 流的帧范围与时刻(MSM 历元时间,周数按 ref_unix 就近确定;非 MSM 帧无时间)。"""
    out: List[FrameSpan] = []
    for f in st.iter_rtcm3_frames(data):
        tow = st.rtcm3_epoch_gpst_tow(f)
        t = None if tow is None else gpst_tow_to_unix_near(tow, ref_unix, leap_s)
        out.append(FrameSpan(start=f.offset, end=f.offset + len(f.raw), t=t))
    return out


def novatel_spans(data: bytes, leap_s: int = 18) -> List[FrameSpan]:
    """NovAtel 二进制流里 CRC 通过的帧的范围与时刻(头部 GPS 周 + 毫秒)。

    CRC 不过的候选帧不作为切分边界(`iter_novatel_frames` 对它只前进 1 字节,范围会与后面的
    真帧重叠);它的字节照样随下一块发出去。
    """
    out: List[FrameSpan] = []
    for f in st.iter_novatel_frames(data):
        if not f.crc_ok:
            continue
        t = st.gpst_to_unix_utc(f.week, f.ms / 1000.0, leap_s)
        out.append(FrameSpan(start=f.offset, end=f.offset + len(f.raw), t=t))
    return out


def inject_periodic(chunks: Sequence[Chunk], payload: bytes, period_s: float, frames: int) -> List[Chunk]:
    """在第一块之前注入一次 payload,之后每隔 period_s 数据秒再注入一次。

    第 k 次注入的时刻 T_k = 第一块时刻 + k·period_s,插在第一个 t ≥ T_k 的数据块之前
    (块边界就是帧边界,不会把星历插进一条帧中间);最后一块之后不再注入。
    """
    if period_s <= 0:
        raise ValueError("period_s 必须 > 0")
    if not chunks:
        return []
    first_t = chunks[0].t
    out: List[Chunk] = [Chunk(t=first_t, data=payload, frames=frames, kind="nav")]
    k = 1
    for c in chunks:
        while first_t + k * period_s <= c.t:
            out.append(Chunk(t=first_t + k * period_s, data=payload, frames=frames, kind="nav"))
            k += 1
        out.append(c)
    return out


def truncate(chunks: Sequence[Chunk], t_end: Optional[float]) -> List[Chunk]:
    """只保留 t < t_end 的块(试跑 --duration-s);t_end 为 None 时全保留。"""
    if t_end is None:
        return list(chunks)
    return [c for c in chunks if c.t < t_end]


def wall_due(t: float, t0: float, w0: float, speed: float) -> float:
    """数据时刻 t 的发送墙钟(monotonic)时刻:W0 + (t − t0)/speed。"""
    return w0 + (t - t0) / speed


def data_time(wall: float, t0: float, w0: float, speed: float) -> float:
    """墙钟(monotonic)时刻对应的数据时刻,/clock 发布用:t0 + (wall − W0)·speed。"""
    return t0 + (wall - w0) * speed


# ---------- 等下游(rtkrcv)连上再开始 ----------
#
# rtkrcv_node 的 LocalReserver 对没有连接的时段直接丢字节,所以回放要等 rtkrcv 的两路 tcpcli 连上
# rtkrcv_node 的本机端口(corr_port/obs_port)才开始发。
# 历史:2026-09-16 dryB 试跑时 conf 还没写 misc-timeout,rtkrcv 用默认 10 s 空闲断开、10 s 重连,
# 连接"连 10 s、断 10 s"循环,回放开头落在断开窗口里丢了 5 s 观测和第一次星历注入,当时这里改成
# 等一次"先断后连"的新连接。gnss_bringup Task 6 F2 给 conf 写了 misc-timeout=0 之后连接不再空闲断开,
# 旧的等法永远等不到新连接(实测 dryB 等满 25 s 退出 3),改为只要求"此刻已连着"。

_TCP_ESTABLISHED = 0x01


def established_local_ports(proc_net_tcp: str) -> Dict[int, int]:
    """解析 /proc/net/tcp(或 tcp6)文本,按本地端口统计 ESTABLISHED 状态的套接字数。"""
    counts: Dict[int, int] = {}
    for line in proc_net_tcp.splitlines():
        fields = line.split()
        if len(fields) < 4 or not fields[0].endswith(":"):
            continue
        try:
            local_port = int(fields[1].rsplit(":", 1)[1], 16)
            state = int(fields[3], 16)
        except (IndexError, ValueError):
            continue
        if state == _TCP_ESTABLISHED:
            counts[local_port] = counts.get(local_port, 0) + 1
    return counts


def ports_all_connected(ports: Iterable[int], established_counts: Dict[int, int]) -> bool:
    """ports 里每个端口都至少有一条 ESTABLISHED 连接时返回 True(空列表为 True)。

    用来等下游 rtkrcv 的 tcpcli 连上 rtkrcv_node 的本机端口再开始回放。rtkrcv conf 写了
    misc-timeout=0(gnss_bringup Task 6 F2),连上之后不会因为回放开始前没有数据而空闲断开,
    所以"此刻已连着"就够了。(此前 conf 用 RTKLIB 默认 10 s 空闲断开,这里曾经要等一次"先断后连"。)
    """
    return all(established_counts.get(p, 0) > 0 for p in ports)
