#!/usr/bin/env python3
"""candump_log.py —— candump 格式日志的纯函数解析(Task 2)。

格式例:`(1789462171.002222) can7 320#8409DCC7310C0000`
—— `(unix 时间戳)` `接口名` `CAN-ID#十六进制数据`(标准帧 3 位十六进制 ID,
扩展帧 8 位十六进制 ID)。不做任何 I/O 之外的解析,`first_timestamp` 只读文件。
"""
import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class CanRecord:
    t: float       # unix 时间戳(秒,来自 candump 记录头)
    iface: str     # 接口名,如 "can7"
    can_id: int    # CAN ID(标准帧或扩展帧)
    data: bytes    # 数据段(0-8 字节)


# `(1789462171.002222) can7 320#8409DCC7310C0000`;CAN-ID 3 位(标准帧,11 位)
# 或 8 位(扩展帧,29 位)十六进制,数据段允许为空(DLC=0)。
_LINE_RE = re.compile(
    r"^\(([0-9]+\.[0-9]+)\)\s+(\S+)\s+([0-9A-Fa-f]{3}|[0-9A-Fa-f]{8})#([0-9A-Fa-f]*)$")


def parse_line(line: str) -> Optional[CanRecord]:
    """解析一行 candump 记录;格式不对返回 None(空行、缺字段、非十六进制等)。"""
    s = line.strip()
    if not s:
        return None
    m = _LINE_RE.match(s)
    if not m:
        return None
    ts_str, iface, id_hex, data_hex = m.groups()
    if len(data_hex) % 2 != 0:
        return None
    try:
        t = float(ts_str)
        can_id = int(id_hex, 16)
        data = bytes.fromhex(data_hex)
    except ValueError:
        return None
    return CanRecord(t=t, iface=iface, can_id=can_id, data=data)


def first_timestamp(path: str) -> float:
    """返回文件中第一条能解析成功的记录的时间戳;跳过无法解析的行。"""
    with open(path) as f:
        for line in f:
            rec = parse_line(line)
            if rec is not None:
                return rec.t
    raise ValueError(f"{path} 中没有任何能解析的 candump 记录")
