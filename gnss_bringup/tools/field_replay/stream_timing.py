#!/usr/bin/env python3
"""stream_timing.py —— 现场回放工具的纯函数:RTCM3 / NovAtel 二进制分帧与帧时间(Task 2)。

不依赖 ROS、不做任何 I/O;只从内存中的 bytes 提取帧与时间戳,供
`field_replay.py` 编排器与单元测试复用。

覆盖的三类帧:
- RTCM3(`iter_rtcm3_frames` + `rtcm3_epoch_gpst_tow`):平台差分转发流
  (`base.rtcm3`)与 Task 2 Step 5 生成的星历电文。
- NovAtel OEM7 二进制(`iter_novatel_frames`):610 原始观测/惯导流
  (`cgi610.dat`),长头(AA 44 12)与短头(AA 44 13)两种同步。
- candump 文本行:见同目录 `candump_log.py`。

时间基准换算见 `gpst_to_unix_utc`;多路帧按时间合并调度见 `schedule`。
"""
from dataclasses import dataclass
from typing import Iterable, Iterator, List, Optional, Tuple


@dataclass
class Rtcm3Frame:
    """一条完整且 CRC-24Q 校验通过的 RTCM3 帧。"""
    offset: int   # 帧在输入 bytes 中的起始偏移
    raw: bytes    # 完整帧字节(含 3 字节头 + payload + 3 字节 CRC)
    msg_type: int  # RTCM 消息类型号(payload 前 12 位)
    crc_ok: bool  # 恒为 True(CRC 不过的候选帧不会被产出,见 iter_rtcm3_frames)


@dataclass
class NovatelFrame:
    """一条 NovAtel OEM7 二进制帧(长头或短头),不论 CRC 是否通过都会产出。"""
    offset: int
    raw: bytes
    msg_id: int
    week: int
    ms: int
    crc_ok: bool


_CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    """RTCM3/SBAS 用的 CRC-24Q(多项式 0x1864CFB,初值 0),逐位实现,
    算法与 RTKLIB rtkcmn.c 的 `rtk_crc24q`(查表版,同一多项式/初值)等价。"""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= _CRC24Q_POLY
        crc &= 0xFFFFFF
    return crc


_RTCM3_PREAMBLE = 0xD3

# MSM1-7 消息号范围,按卫星系统分组(RTCM 10403.x 标准编号)。
_MSM_GPS = range(1071, 1078)
_MSM_GLO = range(1081, 1088)
_MSM_GAL = range(1091, 1098)
_MSM_SBS = range(1101, 1108)
_MSM_QZS = range(1111, 1118)
_MSM_BDS = range(1121, 1128)


def _bits(data: bytes, bitpos: int, nbits: int) -> int:
    """从大端位流(MSB 在前,RTCM/NovAtel 头部字段的打包方式)里,从 bitpos 开始
    取 nbits 位,返回无符号整数。"""
    v = 0
    for i in range(nbits):
        byte_idx = (bitpos + i) // 8
        bit_idx = 7 - ((bitpos + i) % 8)
        v = (v << 1) | ((data[byte_idx] >> bit_idx) & 1)
    return v


def iter_rtcm3_frames(data: bytes) -> Iterator[Rtcm3Frame]:
    """按 0xD3 前导 + 10 位长度分帧,校验 CRC-24Q;CRC 不过时该候选帧不产出,
    从下一字节继续找同步(不是跳过整帧长度,因为长度域本身可能就是被污染的
    数据,只有向前挪 1 字节重新找 0xD3 才安全)。"""
    n = len(data)
    i = 0
    while i < n:
        if data[i] != _RTCM3_PREAMBLE:
            i += 1
            continue
        if i + 3 > n:
            i += 1
            continue
        length = ((data[i + 1] & 0x03) << 8) | data[i + 2]
        frame_len = 3 + length + 3
        if i + frame_len > n:
            i += 1
            continue
        frame = data[i:i + frame_len]
        crc_calc = crc24q(frame[:3 + length])
        crc_recv = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
        if crc_calc != crc_recv:
            i += 1
            continue
        msg_type = (frame[3] << 4) | (frame[4] >> 4)
        yield Rtcm3Frame(offset=i, raw=bytes(frame), msg_type=msg_type, crc_ok=True)
        i += frame_len


def rtcm3_epoch_gpst_tow(frame: Rtcm3Frame) -> Optional[float]:
    """MSM1-7(1071-1127)取帧内历元时间,转换为 GPST 周内秒;非 MSM 帧返回 None。

    比特布局(RTCM 10403.x MSM 头,紧跟 3 字节帧头之后):
    DF002 消息号(12 位)+ DF003 测站 ID(12 位)+ 历元时间。
    GPS/Galileo/QZSS/SBAS 历元时间是 30 位周内毫秒,直接除 1000。
    BDS 历元时间是 30 位 BDT 周内毫秒,BDT = GPST - 14s,故 GPST = BDT + 14。
    GLONASS 历元时间是 3 位 DF416"莫斯科日内星期"+ 27 位日内毫秒(莫斯科时间);
    莫斯科时间 = UTC + 3h,GPST = UTC + 18s(闰秒),所以
    GPST 日内秒 = 莫斯科日内秒 - 3h + 18s,GPST 周内秒 = 星期 * 86400 + 上式。
    DF416 里星期字段值 7 表示"未知"(没有星期就没法算周内秒),此时返回
    None——按无时间帧处理,交给 schedule() 的继承规则去补时间。

    所有分支的结果都按 % 604800 折回 [0, 604800) 周内秒:
    BDS/GPS/Galileo/QZSS/SBAS 的原始字段本身理论上不会越界(30 位周内毫秒
    定义域就是一周以内),但字段本身可能因为传输错误或本函数外部误用而
    越界;GLONASS 换算(莫斯科时间 -3h+18s、以及星期*86400 的组合)在周
    边界附近(周日凌晨前后)会产生负数或超过一周的中间值,必须折回。
    Python 的 `%` 对浮点数取模保证结果非负(不同于 C 的 fmod)。
    """
    msg_type = frame.msg_type
    payload = frame.raw[3:-3]
    header_bits = 24  # DF002(12) + DF003(12)

    if msg_type in _MSM_GLO:
        dow = _bits(payload, header_bits, 3)
        if dow == 7:
            return None  # DF416: 7 = 未知星期,无法定位到周内秒
        tod_ms_moscow = _bits(payload, header_bits + 3, 27)
        tod_s_gpst = tod_ms_moscow / 1000.0 - 3 * 3600.0 + 18.0
        return (dow * 86400.0 + tod_s_gpst) % 604800.0

    if msg_type in _MSM_GPS or msg_type in _MSM_GAL or msg_type in _MSM_QZS or msg_type in _MSM_SBS:
        tow_ms = _bits(payload, header_bits, 30)
        return (tow_ms / 1000.0) % 604800.0

    if msg_type in _MSM_BDS:
        bdt_tow_ms = _bits(payload, header_bits, 30)
        return (bdt_tow_ms / 1000.0 + 14.0) % 604800.0

    return None


_NOVATEL_SYNC1 = 0xAA
_NOVATEL_SYNC2 = 0x44
_NOVATEL_SYNC3_LONG = 0x12
_NOVATEL_SYNC3_SHORT = 0x13
_NOVATEL_LONG_HEADER_MIN = 10  # 至少读到 header_len(byte3)+消息体长度(byte8-9)所需字节数
_NOVATEL_SHORT_HEADER_LEN = 12


def novatel_crc32(data: bytes) -> int:
    """NovAtel OEM7 二进制帧用的 CRC-32(多项式 0xEDB88320,初值 0,不做最终异或),
    与 RTKLIB rtkcmn.c 的 `rtk_crc32` 等价(NovAtel OEMV 固件手册 1.7 节)。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc & 0xFFFFFFFF


def iter_novatel_frames(data: bytes) -> Iterator[NovatelFrame]:
    """按长头(AA 44 12)/短头(AA 44 13)同步分帧,CRC-32 校验结果记入 crc_ok
    (不论 CRC 是否通过都产出——两种头部都用显式长度字段定界,不像 RTCM3 那样
    完全跳过 CRC 不过的候选帧)。

    但扫描位置的前进量必须看 CRC 是否通过:CRC 通过时才按声明的 total_len
    前跳;CRC 不过时只挪 1 字节继续找同步(与 iter_rtcm3_frames 的重同步
    策略一致)。原因:一个"假同步"(在其他数据里偶然出现的 AA 44 12/13)
    后面跟着的长度字段本身也是随机数据,声明的 total_len 可能远大于真实
    情况;如果无条件按 total_len 前跳,会把紧跟在假同步后面的一条真帧
    整个跳过去,永远发现不了。

    长头布局核实自 RTKLIB novatel.c 的 `decode_oem4`/`input_oem4`:
    msg_id@[4:6)(u16 LE)、消息体长度@[8:10)(u16 LE)、周@[14:16)(u16 LE)、
    毫秒@[16:20)(u32 LE);头长字段在 byte[3],RTKLIB 本身按固定 28 处理,
    这里改为如实读取 byte[3](真实数据里也是 28,两者等价,更通用)。
    CRC-32 覆盖头+消息体,4 字节小端紧跟其后。

    短头布局:task-2-brief.md 给的偏移(周@4-5、毫秒@6-9)缺 msg_id 位置,
    且与 NovAtel OEM7 短头规范不符;已用 `<seg>/raw/cgi610.dat` 第一条真实帧
    核实,采用 msg_id@[4:6)(u16 LE)、周@[6:8)(u16 LE)、毫秒@[8:12)(u32 LE)、
    头长固定 12,byte[3] 为消息体长度(u8)。详见任务报告的订正说明。
    """
    n = len(data)
    i = 0
    while i < n:
        if not (i + 3 <= n and data[i] == _NOVATEL_SYNC1 and data[i + 1] == _NOVATEL_SYNC2):
            i += 1
            continue
        sync3 = data[i + 2]
        if sync3 == _NOVATEL_SYNC3_LONG:
            if i + _NOVATEL_LONG_HEADER_MIN > n:
                i += 1
                continue
            header_len = data[i + 3]
            if header_len < _NOVATEL_LONG_HEADER_MIN or i + header_len > n:
                i += 1
                continue
            if i + 20 > n:  # 还需要读 week@[14:16)/ms@[16:20),这两个字段固定在长头内
                i += 1
                continue
            body_len = int.from_bytes(data[i + 8:i + 10], "little")
            total_len = header_len + body_len + 4
            if i + total_len > n:
                i += 1
                continue
            msg_id = int.from_bytes(data[i + 4:i + 6], "little")
            week = int.from_bytes(data[i + 14:i + 16], "little")
            ms = int.from_bytes(data[i + 16:i + 20], "little")
        elif sync3 == _NOVATEL_SYNC3_SHORT:
            if i + _NOVATEL_SHORT_HEADER_LEN > n:
                i += 1
                continue
            body_len = data[i + 3]
            total_len = _NOVATEL_SHORT_HEADER_LEN + body_len + 4
            if i + total_len > n:
                i += 1
                continue
            msg_id = int.from_bytes(data[i + 4:i + 6], "little")
            week = int.from_bytes(data[i + 6:i + 8], "little")
            ms = int.from_bytes(data[i + 8:i + 12], "little")
        else:
            i += 1
            continue

        frame = data[i:i + total_len]
        crc_calc = novatel_crc32(frame[:-4])
        crc_recv = int.from_bytes(frame[-4:], "little")
        ok = crc_calc == crc_recv
        yield NovatelFrame(offset=i, raw=bytes(frame), msg_id=msg_id, week=week, ms=ms,
                            crc_ok=ok)
        i += total_len if ok else 1


_GPS_EPOCH_UNIX = 315964800.0  # 1980-01-06 00:00:00 UTC


def gpst_to_unix_utc(week: int, tow_s: float, leap_s: int = 18) -> float:
    """GPST(周 + 周内秒)转 unix UTC 秒(固定减 leap_s 闰秒)。"""
    return _GPS_EPOCH_UNIX + week * 604800.0 + tow_s - leap_s


def schedule(frames_with_time: Iterable[Tuple[Optional[float], bytes]]) -> List[Tuple[float, bytes]]:
    """按时间为帧序列补齐:无时间的帧沿用前一个有时间的帧;开头的无时间帧
    沿用第一个有时间的帧。"""
    items = list(frames_with_time)
    first_known = next((t for t, _ in items if t is not None), None)
    if first_known is None:
        raise ValueError("frames_with_time 中没有任何带时间的帧,无法调度")
    last = first_known
    out: List[Tuple[float, bytes]] = []
    for t, raw in items:
        if t is not None:
            last = t
        out.append((last, raw))
    return out
