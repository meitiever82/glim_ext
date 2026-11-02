# Plan 1: 数据接入与落盘 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 四路 GNSS 数据（差分 RTCM、板卡原始观测、纯卫导解 GPCHC、CAN 融合输出）的采集、解析、裸流落盘与断线事件记录，附数据源模拟器，可独立上车运行录数据。

**Architecture:** 单 Python asyncio 进程。采集协程只做"收→落盘→回调"；解析器为纯函数/纯类（无 IO）；存储层负责小时滚动裸流 + sidecar 索引 + SQLite 事件表；本地转发服务器为 Plan 2 的 rtkrcv 预留输入。

**Tech Stack:** Python 3.11+ / asyncio、pyyaml、python-can、sqlite3（标准库）、pytest + pytest-asyncio。

**Spec:** `docs/superpowers/specs/2026-08-31-rtk-monitor-design.md`（本计划实现其 §2.1 collector/parser/store 部分、§3 全部、§8 模拟器）

## Global Constraints

- Python >= 3.11；运行时第三方依赖仅 `pyyaml`、`python-can`；dev 依赖仅 `pytest`、`pytest-asyncio`。
- src 布局：代码在 `src/rtk_monitor/`，测试在 `tests/`，包内代码与注释一律英文。
- 解析器不做任何 IO；采集协程不做任何解析——层间只传 `bytes` 与回调。
- 落盘裸流必须与线上字节完全一致（不加壳、不重排），目录 `<data_root>/YYYYMMDD/`，文件按小时。
- 每个 commit 信息末尾带：`Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。
- pytest 配置 `asyncio_mode = "auto"`（async 测试函数免装饰器）。

---

### Task 1: 项目脚手架与配置加载

**Files:**
- Create: `pyproject.toml`
- Create: `config.yaml.example`
- Create: `src/rtk_monitor/__init__.py`（空文件）
- Create: `src/rtk_monitor/config.py`
- Test: `tests/test_config.py`

**Interfaces:**
- Produces: `load_config(path: str | Path) -> Config`；`Config` dataclass 字段见实现，后续所有 Task 从它取参数。

- [ ] **Step 1: 写 pyproject.toml 与 config.yaml.example**

```toml
# pyproject.toml
[project]
name = "rtk-monitor"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["pyyaml>=6.0", "python-can>=4.0"]

[project.optional-dependencies]
dev = ["pytest>=8.0", "pytest-asyncio>=0.23"]

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["src"]

[tool.pytest.ini_options]
asyncio_mode = "auto"
testpaths = ["tests"]
```

```yaml
# config.yaml.example — copy to config.yaml and edit for the vehicle
data_root: /data/gnsslog
db_path: /data/gnsslog/rtk_monitor.db

corrections:      # route 1: platform RTCM broadcast
  host: 192.168.10.1
  port: 6001
raw_obs:          # route 2: CGI-610 TCP Server7 (GNSS board raw data only)
  host: 192.168.200.1
  port: 9901
gnss_solution:    # route 3: CGI-610 Client7 GPCHC text stream (we listen)
  host: 0.0.0.0
  port: 9902
  listen: true    # true = accept connection from 610, false = connect out

can_channel: can0

reserve:          # local re-serve ports for rtkrcv (Plan 2)
  corrections_port: 15010
  raw_obs_port: 15011

retention_days: 14
disk_watermark_pct: 85.0
```

- [ ] **Step 2: 写失败测试**

```python
# tests/test_config.py
from pathlib import Path
from rtk_monitor.config import load_config

EXAMPLE = Path(__file__).resolve().parents[1] / "config.yaml.example"

def test_load_example_config():
    cfg = load_config(EXAMPLE)
    assert cfg.data_root == Path("/data/gnsslog")
    assert cfg.corrections.host == "192.168.10.1"
    assert cfg.corrections.port == 6001
    assert cfg.corrections.listen is False          # default
    assert cfg.gnss_solution.listen is True
    assert cfg.can_channel == "can0"
    assert cfg.reserve_corrections_port == 15010
    assert cfg.reserve_raw_obs_port == 15011
    assert cfg.retention_days == 14
    assert cfg.disk_watermark_pct == 85.0

def test_missing_key_raises(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("data_root: /tmp/x\n")
    try:
        load_config(p)
        assert False, "should raise"
    except KeyError:
        pass
```

- [ ] **Step 3: 运行确认失败**

Run: `pip install -e ".[dev]" && pytest tests/test_config.py -v`
Expected: FAIL（`ModuleNotFoundError: rtk_monitor.config`）

- [ ] **Step 4: 实现 config.py**

```python
# src/rtk_monitor/config.py
"""Load and validate config.yaml into typed dataclasses."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class StreamCfg:
    host: str
    port: int
    listen: bool = False


@dataclass(frozen=True)
class Config:
    data_root: Path
    db_path: Path
    corrections: StreamCfg
    raw_obs: StreamCfg
    gnss_solution: StreamCfg
    can_channel: str
    reserve_corrections_port: int
    reserve_raw_obs_port: int
    retention_days: int
    disk_watermark_pct: float


def _stream(d: dict) -> StreamCfg:
    return StreamCfg(host=d["host"], port=int(d["port"]), listen=bool(d.get("listen", False)))


def load_config(path: str | Path) -> Config:
    raw = yaml.safe_load(Path(path).read_text())
    return Config(
        data_root=Path(raw["data_root"]),
        db_path=Path(raw["db_path"]),
        corrections=_stream(raw["corrections"]),
        raw_obs=_stream(raw["raw_obs"]),
        gnss_solution=_stream(raw["gnss_solution"]),
        can_channel=raw["can_channel"],
        reserve_corrections_port=int(raw["reserve"]["corrections_port"]),
        reserve_raw_obs_port=int(raw["reserve"]["raw_obs_port"]),
        retention_days=int(raw["retention_days"]),
        disk_watermark_pct=float(raw["disk_watermark_pct"]),
    )
```

- [ ] **Step 5: 运行确认通过**

Run: `pytest tests/test_config.py -v`
Expected: 2 PASS

- [ ] **Step 6: Commit**

```bash
git add pyproject.toml config.yaml.example src/rtk_monitor tests/test_config.py
git commit -m "feat: project scaffold and typed config loader"
```

---

### Task 2: RTCM3 帧切分与 1005 基站坐标解析

**Files:**
- Create: `src/rtk_monitor/parsers/__init__.py`（空）
- Create: `src/rtk_monitor/parsers/rtcm.py`
- Test: `tests/test_rtcm.py`

**Interfaces:**
- Produces: `RtcmFramer.feed(data: bytes) -> list[RtcmMessage]`（`RtcmMessage(msg_type: int, payload: bytes, raw: bytes)`，`framer.crc_errors: int`）；`parse_1005(payload: bytes) -> tuple[float, float, float]`（ECEF 米）；`crc24q(data: bytes) -> int`。Task 7 用 framer 提取消息类型写 sidecar；Plan 2 诊断规则 2 用 `parse_1005`。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_rtcm.py
from rtk_monitor.parsers.rtcm import RtcmFramer, crc24q, parse_1005


def _frame(payload: bytes) -> bytes:
    head = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF])
    body = head + payload
    return body + crc24q(body).to_bytes(3, "big")


def _payload_msgtype(msg_type: int, tail_bits: int) -> bytes:
    """Payload whose first 12 bits are msg_type, rest zeros."""
    n_bytes = (12 + tail_bits + 7) // 8
    v = msg_type << (n_bytes * 8 - 12)
    return v.to_bytes(n_bytes, "big")


def _payload_1005(x: float, y: float, z: float) -> bytes:
    """152-bit type-1005 payload with given ECEF coords (meters)."""
    def enc(val: float) -> int:
        i = round(val / 1e-4)
        return i & ((1 << 38) - 1)
    bits = 0
    bits |= 1005 << (152 - 12)          # DF002 message number
    bits |= enc(x) << (152 - 34 - 38)   # ECEF-X at bit 34
    bits |= enc(y) << (152 - 74 - 38)   # ECEF-Y at bit 74
    bits |= enc(z) << (152 - 114 - 38)  # ECEF-Z at bit 114
    return bits.to_bytes(19, "big")


def test_single_frame():
    payload = _payload_msgtype(1074, tail_bits=52)
    msgs = RtcmFramer().feed(_frame(payload))
    assert len(msgs) == 1
    assert msgs[0].msg_type == 1074
    assert msgs[0].raw == _frame(payload)


def test_split_across_chunks_and_garbage_prefix():
    f = RtcmFramer()
    frame = _frame(_payload_msgtype(1124, 52))
    assert f.feed(b"\x00garbage" + frame[:5]) == []
    msgs = f.feed(frame[5:] + frame)  # remainder + a second full frame
    assert [m.msg_type for m in msgs] == [1124, 1124]


def test_crc_error_skips_byte_not_frame():
    f = RtcmFramer()
    frame = bytearray(_frame(_payload_msgtype(1005, 140)))
    frame[10] ^= 0xFF  # corrupt
    good = _frame(_payload_msgtype(1084, 52))
    msgs = f.feed(bytes(frame) + good)
    assert [m.msg_type for m in msgs] == [1084]
    assert f.crc_errors >= 1


def test_parse_1005_roundtrip():
    x, y, z = -2148744.1234, 4426641.2345, 4044655.9876
    px, py, pz = parse_1005(_payload_1005(x, y, z))
    assert abs(px - x) < 1e-4 and abs(py - y) < 1e-4 and abs(pz - z) < 1e-4
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_rtcm.py -v`
Expected: FAIL（模块不存在）

- [ ] **Step 3: 实现 rtcm.py**

```python
# src/rtk_monitor/parsers/rtcm.py
"""Incremental RTCM3 frame splitter and message-1005 station coordinates."""
from __future__ import annotations

from dataclasses import dataclass

CRC24Q_POLY = 0x1864CFB


def crc24q(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF


@dataclass(frozen=True)
class RtcmMessage:
    msg_type: int
    payload: bytes
    raw: bytes


class RtcmFramer:
    """Feed arbitrary byte chunks; get complete CRC-checked RTCM3 messages."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[RtcmMessage]:
        self._buf.extend(data)
        out: list[RtcmMessage] = []
        while True:
            start = self._buf.find(b"\xd3")
            if start < 0:
                self._buf.clear()
                break
            if start:
                del self._buf[:start]
            if len(self._buf) < 6:
                break
            length = ((self._buf[1] & 0x03) << 8) | self._buf[2]
            total = 3 + length + 3
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                payload = frame[3:-3]
                msg_type = (payload[0] << 4) | (payload[1] >> 4)
                out.append(RtcmMessage(msg_type, payload, frame))
                del self._buf[:total]
            else:
                self.crc_errors += 1
                del self._buf[:1]
        return out


def _get_bits(data: bytes, start: int, length: int) -> int:
    value = 0
    for i in range(start, start + length):
        value = (value << 1) | ((data[i // 8] >> (7 - i % 8)) & 1)
    return value


def _get_sbits(data: bytes, start: int, length: int) -> int:
    v = _get_bits(data, start, length)
    return v - (1 << length) if v & (1 << (length - 1)) else v


def parse_1005(payload: bytes) -> tuple[float, float, float]:
    """Return base-station ECEF (x, y, z) in meters from a 1005 payload."""
    x = _get_sbits(payload, 34, 38) * 1e-4
    y = _get_sbits(payload, 74, 38) * 1e-4
    z = _get_sbits(payload, 114, 38) * 1e-4
    return x, y, z
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_rtcm.py -v`
Expected: 4 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/parsers tests/test_rtcm.py
git commit -m "feat: RTCM3 framer with CRC24Q and 1005 base coordinates parser"
```

---

### Task 3: GPCHC 卫导解文本解析

**Files:**
- Create: `src/rtk_monitor/parsers/gpchc.py`
- Test: `tests/test_gpchc.py`

**Interfaces:**
- Produces: `LineFramer.feed(data: bytes) -> list[str]`；`parse_gpchc(line: str) -> GpchcEpoch | None`（校验失败/非 GPCHC 返回 None）。`GpchcEpoch` 字段：`week:int, tow:float, heading:float, pitch:float, roll:float, lat:float, lon:float, alt:float, ve:float, vn:float, vu:float, speed:float, nsv1:int, nsv2:int, sat_status:int, sys_state:int, diff_age:float`。Plan 2 历元表直接消费。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_gpchc.py
from rtk_monitor.parsers.gpchc import GpchcEpoch, LineFramer, parse_gpchc

BODY = ("GPCHC,2372,113755.36,174.20,1.25,-0.80,0.12,-0.05,0.30,"
        "0.0123,-0.0045,0.9987,44.50123456,90.28765432,617.123,"
        "0.02,-0.01,0.00,0.02,39,38,42,1.2,0")


def _mk(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


def test_parse_valid_sentence():
    e = parse_gpchc(_mk(BODY))
    assert isinstance(e, GpchcEpoch)
    assert e.week == 2372 and abs(e.tow - 113755.36) < 1e-6
    assert abs(e.lat - 44.50123456) < 1e-9
    assert abs(e.heading - 174.20) < 1e-6
    assert e.nsv1 == 39 and e.nsv2 == 38
    # status "42": high nibble = satellite status (4 = RTK fixed + heading),
    # low nibble = system state (2 = INS/GNSS integrated), per CGI-610 manual
    assert e.sat_status == 4 and e.sys_state == 2
    assert abs(e.diff_age - 1.2) < 1e-6


def test_bad_checksum_returns_none():
    assert parse_gpchc(f"${BODY}*00") is None


def test_other_sentence_returns_none():
    assert parse_gpchc(_mk("GPGGA,1,2,3")) is None


def test_line_framer_reassembles_chunks():
    f = LineFramer()
    s = _mk(BODY) + "\r\n"
    assert f.feed(s[:10].encode()) == []
    lines = f.feed((s[10:] + s).encode())
    assert lines == [_mk(BODY), _mk(BODY)]
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_gpchc.py -v`
Expected: FAIL（模块不存在）

- [ ] **Step 3: 实现 gpchc.py**

```python
# src/rtk_monitor/parsers/gpchc.py
"""Parse Huace $GPCHC integrated-navigation sentences (CGI-610 'satnav data')."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GpchcEpoch:
    week: int
    tow: float
    heading: float
    pitch: float
    roll: float
    lat: float
    lon: float
    alt: float
    ve: float
    vn: float
    vu: float
    speed: float
    nsv1: int
    nsv2: int
    sat_status: int
    sys_state: int
    diff_age: float


class LineFramer:
    """Reassemble a TCP byte stream into complete text lines."""

    def __init__(self) -> None:
        self._buf = b""

    def feed(self, data: bytes) -> list[str]:
        self._buf += data
        *lines, self._buf = self._buf.split(b"\n")
        return [ln.strip().decode("ascii", "replace") for ln in lines if ln.strip()]


def parse_gpchc(line: str) -> GpchcEpoch | None:
    if not line.startswith("$GPCHC,") or "*" not in line:
        return None
    body, cs_str = line[1:].rsplit("*", 1)
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    try:
        if cs != int(cs_str, 16):
            return None
        f = body.split(",")
        status = int(f[21], 16)
        return GpchcEpoch(
            week=int(f[1]), tow=float(f[2]),
            heading=float(f[3]), pitch=float(f[4]), roll=float(f[5]),
            lat=float(f[12]), lon=float(f[13]), alt=float(f[14]),
            ve=float(f[15]), vn=float(f[16]), vu=float(f[17]), speed=float(f[18]),
            nsv1=int(f[19]), nsv2=int(f[20]),
            sat_status=(status >> 4) & 0xF, sys_state=status & 0xF,
            diff_age=float(f[22]),
        )
    except (IndexError, ValueError):
        return None
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_gpchc.py -v`
Expected: 4 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/parsers/gpchc.py tests/test_gpchc.py
git commit -m "feat: GPCHC sentence parser with NMEA checksum and line framer"
```

**注意**：字段序号基于 CGI-610 手册 GPCHC 协议（week=f[1] … Warning=f[23]）。第一次接到 610 真实流时，若字段错位以实测为准修正测试样例——手册与固件版本可能有差异（§11 风险项）。

---

### Task 4: CAN 融合输出解码（移植 decode_cgi610.py）

**Files:**
- Create: `src/rtk_monitor/parsers/cgi610_can.py`
- Test: `tests/test_cgi610_can.py`

**Interfaces:**
- Produces: `Cgi610Assembler.feed(can_id: int, data: bytes, host_time: float) -> NavCycle | None`（每个 50Hz 周期在下一个 0x320 到来时产出）。`NavCycle` 字段见实现。Plan 2 历元表消费。

**背景**：解码逻辑与 `/home/steve/deploy_ws/issues/gnss/can_record/decode_cgi610.py` 一致（该脚本已用真实数据验证：字节序小端，0x320 为周期起点）。本 Task 把它从"读 candump 文件写 CSV"重构为"纯增量解码类"。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_cgi610_can.py
import struct

from rtk_monitor.parsers.cgi610_can import Cgi610Assembler


def _three_s20(a: float, b: float, c: float, scale: float) -> bytes:
    def enc(v: float) -> int:
        return round(v / scale) & 0xFFFFF
    u = enc(a) | (enc(b) << 20) | (enc(c) << 40)
    return u.to_bytes(8, "little")


def _cycle_frames():
    """One complete 50 Hz cycle as (can_id, data) pairs."""
    yield 0x320, struct.pack("<HI", 2372, 113755360) + b"\x00\x00"   # week, tow*1e3
    yield 0x321, _three_s20(1.23, -0.50, 0.07, 0.01)                  # gyro dps
    yield 0x322, _three_s20(0.0123, -0.0045, 0.9987, 0.0001)          # accel g
    yield 0x323, bytes([2, 39, 4, 38]) + struct.pack("<H", 120) + bytes([40, 41])
    yield 0x325, struct.pack("<i", 617123) + b"\x00" * 4              # alt mm
    yield 0x326, _three_s20(0.0112, 0.0108, 0.0322, 0.0001)           # pos sigma
    yield 0x327, struct.pack("<4h", 210, -15, 3, 211)                 # vel cm/s
    yield 0x328, struct.pack("<4H", 25, 24, 60, 26)                   # vel sigma mm/s
    yield 0x329, _three_s20(0.01, -0.02, 0.001, 0.0001)               # veh accel g
    yield 0x32A, struct.pack("<H", 17420) + struct.pack("<2h", 125, -80) + b"\x00\x00"
    yield 0x32B, _three_s20(0.1115, 0.05, 0.05, 0.0001)               # att sigma
    yield 0x32C, _three_s20(0.5, -0.2, 1.1, 0.01)                     # ang rate dps
    yield 0x32D, struct.pack("<q", round(90.28765432 / 1e-8))         # lon
    yield 0x32E, struct.pack("<q", round(44.50123456 / 1e-8))         # lat


def test_complete_cycle_emitted_on_next_320():
    asm = Cgi610Assembler()
    for cid, data in _cycle_frames():
        assert asm.feed(cid, data, host_time=100.0) is None
    cyc = asm.feed(0x320, struct.pack("<HI", 2372, 113755380) + b"\x00\x00", 100.02)
    assert cyc is not None
    assert cyc.week == 2372 and abs(cyc.tow - 113755.360) < 1e-6
    assert abs(cyc.lat - 44.50123456) < 1e-8 and abs(cyc.lon - 90.28765432) < 1e-8
    assert abs(cyc.alt - 617.123) < 1e-6
    assert cyc.sys_state == 2 and cyc.sat_status == 4
    assert cyc.sats_used == 39 and abs(cyc.diff_age - 1.2) < 1e-6
    assert abs(cyc.heading - 174.20) < 1e-6 and abs(cyc.pitch - 1.25) < 1e-6
    assert abs(cyc.gyro[0] - 1.23) < 1e-6 and abs(cyc.accel[2] - 0.9987) < 1e-6
    assert abs(cyc.pos_sigma[2] - 0.0322) < 1e-6
    assert abs(cyc.vel[0] - 2.10) < 1e-6
    assert cyc.host_time == 100.0


def test_incomplete_cycle_dropped():
    asm = Cgi610Assembler()
    frames = list(_cycle_frames())[:5]          # missing most IDs
    for cid, data in frames:
        asm.feed(cid, data, 100.0)
    assert asm.feed(0x320, frames[0][1], 100.02) is None
    assert asm.incomplete == 1
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_cgi610_can.py -v`
Expected: FAIL（模块不存在）

- [ ] **Step 3: 实现 cgi610_can.py**

```python
# src/rtk_monitor/parsers/cgi610_can.py
"""Incremental decoder for CGI-610 CAN 2.0 output (little-endian, verified on real logs).

A 50 Hz cycle starts at ID 0x320 (time frame); the cycle is emitted when the
next 0x320 arrives and all required IDs were seen.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

_NEED = {0x320, 0x321, 0x322, 0x323, 0x325, 0x326, 0x327, 0x328,
         0x329, 0x32A, 0x32B, 0x32C, 0x32D, 0x32E}


def _s20(v: int) -> int:
    return v - (1 << 20) if v & (1 << 19) else v


def _three_s20(b: bytes, scale: float) -> tuple[float, float, float]:
    u = int.from_bytes(b, "little")
    return (_s20(u & 0xFFFFF) * scale,
            _s20((u >> 20) & 0xFFFFF) * scale,
            _s20((u >> 40) & 0xFFFFF) * scale)


@dataclass(frozen=True)
class NavCycle:
    host_time: float
    week: int
    tow: float
    sys_state: int
    sats_used: int
    sat_status: int
    sats2_used: int
    diff_age: float
    lat: float
    lon: float
    alt: float
    pos_sigma: tuple[float, float, float]      # E, N, U (m)
    vel: tuple[float, float, float, float]     # E, N, U, total (m/s)
    vel_sigma: tuple[float, float, float, float]
    heading: float
    pitch: float
    roll: float
    att_sigma: tuple[float, float, float]      # heading, pitch, roll (deg)
    gyro: tuple[float, float, float]           # dps
    accel: tuple[float, float, float]          # g


class Cgi610Assembler:
    def __init__(self) -> None:
        self._cur: dict[int, object] | None = None
        self.incomplete = 0

    def feed(self, can_id: int, data: bytes, host_time: float) -> NavCycle | None:
        if can_id == 0x320:
            done = self._flush()
            self._cur = {0x320: (struct.unpack("<H", data[0:2])[0],
                                 struct.unpack("<I", data[2:6])[0] * 0.001),
                         "host": host_time}
            return done
        if self._cur is None:
            return None
        c = self._cur
        if can_id == 0x321:
            c[can_id] = _three_s20(data, 0.01)
        elif can_id in (0x322, 0x329):
            c[can_id] = _three_s20(data, 0.0001)
        elif can_id == 0x323:
            c[can_id] = (data[0], data[1], data[2], data[3],
                         struct.unpack("<H", data[4:6])[0] * 0.01)
        elif can_id == 0x325:
            c[can_id] = struct.unpack("<i", data[0:4])[0] * 0.001
        elif can_id == 0x326:
            c[can_id] = _three_s20(data, 0.0001)
        elif can_id == 0x327:
            c[can_id] = tuple(x * 0.01 for x in struct.unpack("<4h", data))
        elif can_id == 0x328:
            c[can_id] = tuple(x * 0.001 for x in struct.unpack("<4H", data))
        elif can_id == 0x32A:
            hd = struct.unpack("<H", data[0:2])[0] * 0.01
            pt, rl = struct.unpack("<2h", data[2:6])
            c[can_id] = (hd, pt * 0.01, rl * 0.01)
        elif can_id == 0x32B:
            c[can_id] = _three_s20(data, 0.0001)
        elif can_id == 0x32C:
            c[can_id] = _three_s20(data, 0.01)
        elif can_id in (0x32D, 0x32E):
            c[can_id] = struct.unpack("<q", data)[0] * 1e-8
        return None

    def _flush(self) -> NavCycle | None:
        c, self._cur = self._cur, None
        if c is None:
            return None
        if not _NEED.issubset(c.keys()):
            self.incomplete += 1
            return None
        week, tow = c[0x320]
        st = c[0x323]
        hd, pt, rl = c[0x32A]
        return NavCycle(
            host_time=c["host"], week=week, tow=tow,
            sys_state=st[0], sats_used=st[1], sat_status=st[2],
            sats2_used=st[3], diff_age=st[4],
            lat=c[0x32E], lon=c[0x32D], alt=c[0x325],
            pos_sigma=c[0x326], vel=c[0x327], vel_sigma=c[0x328],
            heading=hd, pitch=pt, roll=rl, att_sigma=c[0x32B],
            gyro=c[0x321], accel=c[0x322],
        )
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_cgi610_can.py -v`
Expected: 2 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/parsers/cgi610_can.py tests/test_cgi610_can.py
git commit -m "feat: incremental CGI-610 CAN cycle decoder (ported from validated script)"
```

---

### Task 5: 裸流落盘（小时滚动 + sidecar 索引）

**Files:**
- Create: `src/rtk_monitor/storage/__init__.py`（空）
- Create: `src/rtk_monitor/storage/rawlog.py`
- Test: `tests/test_rawlog.py`

**Interfaces:**
- Produces: `RawLogWriter(root: Path, stream: str, ext: str = "bin", clock=time.time)`，方法 `append(data: bytes, msg_type: int | str | None = None) -> None`、`close() -> None`。文件 `<root>/YYYYMMDD/<stream>_YYYYMMDD_HH.<ext>`，索引同名 `.idx.jsonl`，每条 `{"t","type","off","len"}`。Task 7/8 消费。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_rawlog.py
import json

from rtk_monitor.storage.rawlog import RawLogWriter


class FakeClock:
    def __init__(self, t: float):
        self.t = t
    def __call__(self) -> float:
        return self.t


def test_append_writes_raw_and_index(tmp_path):
    clock = FakeClock(1756699200.0)  # 2026-09-01 12:00:00 +0800 (local test box TZ-agnostic: just a fixed t)
    w = RawLogWriter(tmp_path, "corr", ext="rtcm3", clock=clock)
    w.append(b"\xd3\x00\x01", msg_type=1074)
    w.append(b"\xab\xcd", msg_type=1005)
    w.close()
    days = list(tmp_path.iterdir())
    assert len(days) == 1
    binfile = next(days[0].glob("corr_*.rtcm3"))
    assert binfile.read_bytes() == b"\xd3\x00\x01\xab\xcd"
    idx = [json.loads(l) for l in
           next(days[0].glob("corr_*.idx.jsonl")).read_text().splitlines()]
    assert idx[0] == {"t": 1756699200.0, "type": 1074, "off": 0, "len": 3}
    assert idx[1]["off"] == 3 and idx[1]["len"] == 2


def test_hour_rotation(tmp_path):
    clock = FakeClock(1756699200.0)
    w = RawLogWriter(tmp_path, "corr", clock=clock)
    w.append(b"a")
    clock.t += 3600
    w.append(b"b")
    w.close()
    bins = sorted(p.name for d in tmp_path.iterdir() for p in d.glob("corr_*.bin"))
    assert len(bins) == 2 and bins[0] != bins[1]
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_rawlog.py -v`
Expected: FAIL（模块不存在）

- [ ] **Step 3: 实现 rawlog.py**

```python
# src/rtk_monitor/storage/rawlog.py
"""Hourly-rotated raw byte-stream writer with a JSONL sidecar index.

The raw file is byte-identical to the wire stream so it can be fed directly
to RTKLIB tools (convbin, rnx2rtkp) offline.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import IO


class RawLogWriter:
    def __init__(self, root: Path, stream: str, ext: str = "bin", clock=time.time) -> None:
        self._root = Path(root)
        self._stream = stream
        self._ext = ext
        self._clock = clock
        self._hour_key: str | None = None
        self._file: IO[bytes] | None = None
        self._idx: IO[str] | None = None

    def append(self, data: bytes, msg_type: int | str | None = None) -> None:
        t = self._clock()
        hour_key = time.strftime("%Y%m%d_%H", time.localtime(t))
        if hour_key != self._hour_key:
            self._rotate(hour_key)
        assert self._file is not None and self._idx is not None
        off = self._file.tell()
        self._file.write(data)
        self._idx.write(json.dumps(
            {"t": round(t, 3), "type": msg_type, "off": off, "len": len(data)}) + "\n")
        self._file.flush()
        self._idx.flush()

    def _rotate(self, hour_key: str) -> None:
        self.close()
        day = hour_key[:8]
        d = self._root / day
        d.mkdir(parents=True, exist_ok=True)
        base = d / f"{self._stream}_{hour_key}"
        self._file = open(f"{base}.{self._ext}", "ab")
        self._idx = open(f"{base}.idx.jsonl", "a")
        self._hour_key = hour_key

    def close(self) -> None:
        for f in (self._file, self._idx):
            if f is not None:
                f.close()
        self._file = self._idx = None
        self._hour_key = None
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_rawlog.py -v`
Expected: 2 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/storage tests/test_rawlog.py
git commit -m "feat: hourly-rotated raw log writer with JSONL sidecar index"
```

---

### Task 6: SQLite 事件表

**Files:**
- Create: `src/rtk_monitor/storage/events.py`
- Test: `tests/test_events.py`

**Interfaces:**
- Produces: `EventStore(db_path)`，方法 `record(t: float, etype: str, state: str, detail: str = "") -> int`（返回行 id）、`query(since: float = 0.0) -> list[EventRow]`、`close()`。`EventRow(id, t, etype, state, detail)`。Task 7/8/11 写入断线事件；Plan 2 诊断引擎扩展此表。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_events.py
from rtk_monitor.storage.events import EventStore


def test_record_and_query(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "corrections_link", "disconnected", "retry in 1s")
    assert rid >= 1
    s.record(101.0, "corrections_link", "connected")
    rows = s.query()
    assert len(rows) == 2
    assert rows[0].etype == "corrections_link" and rows[0].state == "disconnected"
    assert s.query(since=100.5)[0].state == "connected"
    s.close()


def test_reopen_persists(tmp_path):
    p = tmp_path / "e.db"
    s = EventStore(p)
    s.record(1.0, "x", "open")
    s.close()
    assert len(EventStore(p).query()) == 1
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_events.py -v`
Expected: FAIL

- [ ] **Step 3: 实现 events.py**

```python
# src/rtk_monitor/storage/events.py
"""SQLite-backed event log. Plan 2's diagnosis engine extends this table."""
from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path

_SCHEMA = """CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL,
    etype TEXT NOT NULL,
    state TEXT NOT NULL,
    detail TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_events_t ON events(t);"""


@dataclass(frozen=True)
class EventRow:
    id: int
    t: float
    etype: str
    state: str
    detail: str


class EventStore:
    def __init__(self, db_path: str | Path) -> None:
        self._db = sqlite3.connect(db_path)
        self._db.executescript(_SCHEMA)
        self._db.commit()

    def record(self, t: float, etype: str, state: str, detail: str = "") -> int:
        cur = self._db.execute(
            "INSERT INTO events (t, etype, state, detail) VALUES (?, ?, ?, ?)",
            (t, etype, state, detail))
        self._db.commit()
        return int(cur.lastrowid)

    def query(self, since: float = 0.0) -> list[EventRow]:
        rows = self._db.execute(
            "SELECT id, t, etype, state, detail FROM events WHERE t >= ? ORDER BY t",
            (since,)).fetchall()
        return [EventRow(*r) for r in rows]

    def close(self) -> None:
        self._db.close()
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_events.py -v`
Expected: 2 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/storage/events.py tests/test_events.py
git commit -m "feat: SQLite event store"
```

---

### Task 7: TCP 采集协程（断线重连 + 事件）

**Files:**
- Create: `src/rtk_monitor/collectors/__init__.py`（空）
- Create: `src/rtk_monitor/collectors/tcp.py`
- Test: `tests/test_tcp_collector.py`

**Interfaces:**
- Consumes: 无（回调由调用方注入）。
- Produces: `TcpCollector(name, host, port, on_data, on_event, listen=False, initial_backoff=1.0, max_backoff=30.0)`，`async run() -> None`（永不返回，`asyncio.CancelledError` 退出）。回调签名：`on_data(data: bytes, host_time: float)`、`on_event(name: str, state: str, detail: str)`，state ∈ {"connected","disconnected"}。`listen=True` 时作为服务端等待对端（610 Client7）连入。Task 11 主装配消费。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_tcp_collector.py
import asyncio

from rtk_monitor.collectors.tcp import TcpCollector


async def _serve_once(port_box: list, payload: bytes, times: int = 2):
    """Server that sends payload then closes, for `times` client connections."""
    remaining = [times]

    async def handler(reader, writer):
        writer.write(payload)
        await writer.drain()
        writer.close()
        remaining[0] -= 1

    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    port_box.append(server.sockets[0].getsockname()[1])
    return server, remaining


async def test_receives_data_and_reconnects():
    port_box: list[int] = []
    server, remaining = await _serve_once(port_box, b"rtcm-bytes")
    got: list[bytes] = []
    events: list[tuple[str, str]] = []
    c = TcpCollector("corr", "127.0.0.1", port_box[0],
                     on_data=lambda d, t: got.append(d),
                     on_event=lambda n, s, det: events.append((n, s)),
                     initial_backoff=0.01, max_backoff=0.05)
    task = asyncio.create_task(c.run())
    for _ in range(200):
        if remaining[0] <= 0 and len(got) >= 2:
            break
        await asyncio.sleep(0.02)
    task.cancel()
    server.close()
    assert b"rtcm-bytes" in got
    assert ("corr", "connected") in events
    assert ("corr", "disconnected") in events
    assert events.count(("corr", "connected")) >= 2  # reconnected after close


async def test_listen_mode_accepts_peer():
    got: list[bytes] = []
    c = TcpCollector("sol", "127.0.0.1", 0, on_data=lambda d, t: got.append(d),
                     on_event=lambda *a: None, listen=True)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)
    assert c.bound_port is not None
    _, writer = await asyncio.open_connection("127.0.0.1", c.bound_port)
    writer.write(b"$GPCHC,...\r\n")
    await writer.drain()
    await asyncio.sleep(0.05)
    writer.close()
    task.cancel()
    assert got == [b"$GPCHC,...\r\n"]
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_tcp_collector.py -v`
Expected: FAIL

- [ ] **Step 3: 实现 tcp.py**

```python
# src/rtk_monitor/collectors/tcp.py
"""TCP stream collector: connect-or-listen, deliver raw chunks, reconnect forever.

The collector performs no parsing — its only jobs are delivering bytes with a
host timestamp and reporting link state transitions.
"""
from __future__ import annotations

import asyncio
import time
from typing import Callable

OnData = Callable[[bytes, float], None]
OnEvent = Callable[[str, str, str], None]


class TcpCollector:
    def __init__(self, name: str, host: str, port: int,
                 on_data: OnData, on_event: OnEvent, listen: bool = False,
                 initial_backoff: float = 1.0, max_backoff: float = 30.0) -> None:
        self._name = name
        self._host = host
        self._port = port
        self._on_data = on_data
        self._on_event = on_event
        self._listen = listen
        self._initial = initial_backoff
        self._max = max_backoff
        self.bound_port: int | None = None

    async def run(self) -> None:
        if self._listen:
            await self._run_server()
        else:
            await self._run_client()

    async def _pump(self, reader: asyncio.StreamReader) -> None:
        self._on_event(self._name, "connected", "")
        while True:
            data = await reader.read(4096)
            if not data:
                break
            self._on_data(data, time.time())

    async def _run_client(self) -> None:
        backoff = self._initial
        while True:
            writer = None
            try:
                reader, writer = await asyncio.open_connection(self._host, self._port)
                backoff = self._initial
                await self._pump(reader)
            except OSError:
                pass
            finally:
                if writer is not None:
                    writer.close()
            self._on_event(self._name, "disconnected", f"retry in {backoff:.0f}s")
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, self._max)

    async def _run_server(self) -> None:
        async def handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
            try:
                await self._pump(reader)
            finally:
                self._on_event(self._name, "disconnected", "peer closed")
                writer.close()

        server = await asyncio.start_server(handler, self._host, self._port)
        self.bound_port = server.sockets[0].getsockname()[1]
        async with server:
            await server.serve_forever()
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_tcp_collector.py -v`
Expected: 2 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/collectors tests/test_tcp_collector.py
git commit -m "feat: TCP collector with reconnect backoff, listen mode and link events"
```

---

### Task 8: CAN 采集与 candump 格式落盘

**Files:**
- Create: `src/rtk_monitor/collectors/can.py`
- Create: `src/rtk_monitor/storage/canlog.py`
- Test: `tests/test_can_collector.py`

**Interfaces:**
- Produces: `CanCollector(bus: can.BusABC, on_frame)`，`async run()`；回调 `on_frame(can_id: int, data: bytes, host_time: float)`。`CandumpWriter(root, channel: str, clock=time.time)`，方法 `append(can_id, data, t)`、`close()`——输出 candump -L 兼容行 `(%.6f) can0 320#AABB...`，按小时滚动（内部复用 Task 5 的 RawLogWriter，ext="log"，不写 sidecar 类型）。Task 11 消费。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_can_collector.py
import asyncio

import can

from rtk_monitor.collectors.can import CanCollector
from rtk_monitor.storage.canlog import CandumpWriter


async def test_collector_receives_virtual_bus_frames():
    with can.Bus(interface="virtual", channel="t0") as tx, \
         can.Bus(interface="virtual", channel="t0") as rx:
        got: list[tuple[int, bytes]] = []
        c = CanCollector(rx, on_frame=lambda i, d, t: got.append((i, d)))
        task = asyncio.create_task(c.run())
        await asyncio.sleep(0.05)
        tx.send(can.Message(arbitration_id=0x320, data=b"\x01\x02", is_extended_id=False))
        for _ in range(100):
            if got:
                break
            await asyncio.sleep(0.01)
        task.cancel()
        assert got == [(0x320, b"\x01\x02")]


def test_candump_writer_format(tmp_path):
    w = CandumpWriter(tmp_path, "can0", clock=lambda: 1756699200.0)
    w.append(0x320, bytes.fromhex("44093c1cc30600aa"), t=1756699200.123456)
    w.close()
    line = next(next(tmp_path.iterdir()).glob("can0_*.log")).read_text().strip()
    assert line == "(1756699200.123456) can0 320#44093C1CC30600AA"
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_can_collector.py -v`
Expected: FAIL

- [ ] **Step 3: 实现 can.py 与 canlog.py**

```python
# src/rtk_monitor/collectors/can.py
"""SocketCAN (or any python-can bus) collector."""
from __future__ import annotations

import asyncio
import time
from typing import Callable

import can

OnFrame = Callable[[int, bytes, float], None]


class CanCollector:
    def __init__(self, bus: can.BusABC, on_frame: OnFrame) -> None:
        self._bus = bus
        self._on_frame = on_frame

    async def run(self) -> None:
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(self._bus, [reader], loop=asyncio.get_running_loop())
        try:
            while True:
                msg = await reader.get_message()
                self._on_frame(msg.arbitration_id, bytes(msg.data),
                               msg.timestamp or time.time())
        finally:
            notifier.stop()
```

```python
# src/rtk_monitor/storage/canlog.py
"""candump -L compatible hourly log, so existing decode scripts keep working."""
from __future__ import annotations

import time
from pathlib import Path

from rtk_monitor.storage.rawlog import RawLogWriter


class CandumpWriter:
    def __init__(self, root: Path, channel: str, clock=time.time) -> None:
        self._channel = channel
        self._raw = RawLogWriter(root, channel, ext="log", clock=clock)

    def append(self, can_id: int, data: bytes, t: float) -> None:
        line = f"({t:.6f}) {self._channel} {can_id:03X}#{data.hex().upper()}\n"
        self._raw.append(line.encode("ascii"))

    def close(self) -> None:
        self._raw.close()
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_can_collector.py -v`
Expected: 2 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/collectors/can.py src/rtk_monitor/storage/canlog.py tests/test_can_collector.py
git commit -m "feat: CAN collector (python-can) and candump-compatible hourly logger"
```

---

### Task 9: 本地转发服务器（rtkrcv 输入预留）

**Files:**
- Create: `src/rtk_monitor/collectors/reserve.py`
- Test: `tests/test_reserve.py`

**Interfaces:**
- Produces: `LocalReserver()`，方法 `async start(port: int, host: str = "127.0.0.1")`（port=0 时看 `bound_port`）、`broadcast(data: bytes)`、`async stop()`。Plan 2 的 rtkrcv 以 tcpcli 连接这两个端口取差分与原始观测。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_reserve.py
import asyncio

from rtk_monitor.collectors.reserve import LocalReserver


async def test_broadcast_to_multiple_clients():
    r = LocalReserver()
    await r.start(0)
    async def client():
        reader, writer = await asyncio.open_connection("127.0.0.1", r.bound_port)
        data = await reader.readexactly(5)
        writer.close()
        return data
    t1, t2 = asyncio.create_task(client()), asyncio.create_task(client())
    await asyncio.sleep(0.05)
    r.broadcast(b"hello")
    assert await t1 == b"hello" and await t2 == b"hello"
    await r.stop()


async def test_broadcast_with_no_clients_is_noop():
    r = LocalReserver()
    await r.start(0)
    r.broadcast(b"x")  # must not raise
    await r.stop()
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_reserve.py -v`
Expected: FAIL

- [ ] **Step 3: 实现 reserve.py**

```python
# src/rtk_monitor/collectors/reserve.py
"""Localhost TCP fan-out: re-serve a collected stream to local consumers (rtkrcv)."""
from __future__ import annotations

import asyncio


class LocalReserver:
    def __init__(self) -> None:
        self._writers: set[asyncio.StreamWriter] = set()
        self._server: asyncio.Server | None = None
        self.bound_port: int | None = None

    async def start(self, port: int, host: str = "127.0.0.1") -> None:
        self._server = await asyncio.start_server(self._on_client, host, port)
        self.bound_port = self._server.sockets[0].getsockname()[1]

    async def _on_client(self, reader: asyncio.StreamReader,
                         writer: asyncio.StreamWriter) -> None:
        self._writers.add(writer)
        try:
            await reader.read()          # block until the client disconnects
        finally:
            self._writers.discard(writer)
            writer.close()

    def broadcast(self, data: bytes) -> None:
        for w in list(self._writers):
            if w.is_closing():
                self._writers.discard(w)
                continue
            w.write(data)

    async def stop(self) -> None:
        for w in list(self._writers):
            w.close()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_reserve.py -v`
Expected: 2 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/collectors/reserve.py tests/test_reserve.py
git commit -m "feat: localhost fan-out server reserving streams for rtkrcv"
```

---

### Task 10: 磁盘清理任务

**Files:**
- Create: `src/rtk_monitor/storage/cleanup.py`
- Test: `tests/test_cleanup.py`

**Interfaces:**
- Produces: `cleanup_logs(root: Path, retention_days: int, watermark_pct: float, disk_usage=shutil.disk_usage, today: datetime.date | None = None) -> list[Path]`（返回被删目录）。规则：目录名为 YYYYMMDD 的日目录，从最旧开始删，直到"最旧目录未超保留天数 且 磁盘使用率低于水位"；当天目录永不删除。Task 11 每小时调用一次。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_cleanup.py
import datetime
from collections import namedtuple

from rtk_monitor.storage.cleanup import cleanup_logs

Usage = namedtuple("Usage", "total used free")
TODAY = datetime.date(2026, 9, 10)


def _mk_days(tmp_path, *days):
    for d in days:
        (tmp_path / d).mkdir()
        (tmp_path / d / "x.bin").write_bytes(b"1")


def test_deletes_beyond_retention(tmp_path):
    _mk_days(tmp_path, "20260820", "20260901", "20260910")
    deleted = cleanup_logs(tmp_path, retention_days=14, watermark_pct=85.0,
                           disk_usage=lambda p: Usage(100, 10, 90), today=TODAY)
    assert [p.name for p in deleted] == ["20260820"]
    assert sorted(p.name for p in tmp_path.iterdir()) == ["20260901", "20260910"]


def test_deletes_oldest_when_over_watermark(tmp_path):
    _mk_days(tmp_path, "20260908", "20260909", "20260910")
    calls = [Usage(100, 90, 10), Usage(100, 80, 20)]  # over, then under after one delete
    deleted = cleanup_logs(tmp_path, retention_days=14, watermark_pct=85.0,
                           disk_usage=lambda p: calls.pop(0), today=TODAY)
    assert [p.name for p in deleted] == ["20260908"]


def test_never_deletes_today(tmp_path):
    _mk_days(tmp_path, "20260910")
    deleted = cleanup_logs(tmp_path, retention_days=0, watermark_pct=0.0,
                           disk_usage=lambda p: Usage(100, 99, 1), today=TODAY)
    assert deleted == []
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_cleanup.py -v`
Expected: FAIL

- [ ] **Step 3: 实现 cleanup.py**

```python
# src/rtk_monitor/storage/cleanup.py
"""Delete oldest day directories by retention age or disk watermark."""
from __future__ import annotations

import datetime
import re
import shutil
from pathlib import Path


def cleanup_logs(root: Path, retention_days: int, watermark_pct: float,
                 disk_usage=shutil.disk_usage,
                 today: datetime.date | None = None) -> list[Path]:
    today = today or datetime.date.today()
    day_dirs = sorted(d for d in Path(root).iterdir()
                      if d.is_dir() and re.fullmatch(r"\d{8}", d.name))
    deleted: list[Path] = []
    for d in day_dirs:
        d_date = datetime.datetime.strptime(d.name, "%Y%m%d").date()
        if d_date >= today:
            break  # never delete today's (or a future-dated) directory
        u = disk_usage(root)
        over_watermark = u.used / u.total * 100 > watermark_pct
        too_old = (today - d_date).days > retention_days
        if not (too_old or over_watermark):
            break
        shutil.rmtree(d)
        deleted.append(d)
    return deleted
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_cleanup.py -v`
Expected: 3 PASS

- [ ] **Step 5: Commit**

```bash
git add src/rtk_monitor/storage/cleanup.py tests/test_cleanup.py
git commit -m "feat: day-directory cleanup by retention and disk watermark"
```

---

### Task 11: 主装配、数据源模拟器与端到端测试

**Files:**
- Create: `src/rtk_monitor/main.py`
- Create: `tools/replay_sources.py`
- Test: `tests/test_e2e.py`

**Interfaces:**
- Consumes: Task 1–10 的全部公开接口（签名见各 Task 的 Interfaces 块）。
- Produces: `build_app(cfg: Config) -> App`、`App.run_forever()`；命令行入口 `python -m rtk_monitor.main <config.yaml>`。`App` 持有 collectors/writers/reserver/EventStore，是 Plan 2 挂载 solver 与诊断的宿主。

- [ ] **Step 1: 写失败测试（端到端）**

```python
# tests/test_e2e.py
import asyncio
import textwrap

import can

from rtk_monitor.config import load_config
from rtk_monitor.main import build_app
from rtk_monitor.parsers.rtcm import crc24q


def _rtcm_frame(msg_type: int) -> bytes:
    payload = (msg_type << 4).to_bytes(2, "big") + b"\x00" * 6
    head = bytes([0xD3, 0x00, len(payload)])
    return head + payload + crc24q(head + payload).to_bytes(3, "big")


async def _stream_server(payload: bytes, interval: float):
    async def handler(reader, writer):
        try:
            while True:
                writer.write(payload)
                await writer.drain()
                await asyncio.sleep(interval)
        except (ConnectionResetError, BrokenPipeError):
            pass
    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    return server, server.sockets[0].getsockname()[1]


async def test_end_to_end_collect_and_log(tmp_path):
    corr_srv, corr_port = await _stream_server(_rtcm_frame(1074), 0.02)
    obs_srv, obs_port = await _stream_server(_rtcm_frame(1077), 0.02)
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: {corr_port}}}
        raw_obs: {{host: 127.0.0.1, port: {obs_port}}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:e2e
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        """))
    app = build_app(load_config(cfg_file))
    task = asyncio.create_task(app.run_forever())
    await asyncio.sleep(0.05)
    with can.Bus(interface="virtual", channel="e2e") as tx:
        tx.send(can.Message(arbitration_id=0x320,
                            data=bytes.fromhex("4409a03c3c060000"), is_extended_id=False))
        await asyncio.sleep(0.5)
    task.cancel()
    await app.shutdown()

    day = next((tmp_path / "log").iterdir())
    assert next(day.glob("corr_*.rtcm3")).stat().st_size > 0
    assert next(day.glob("obs_*.rtcm3")).stat().st_size > 0
    assert "320#" in next(day.glob("can*_*.log")).read_text()
    idx_line = next(day.glob("corr_*.idx.jsonl")).read_text().splitlines()[0]
    assert '"type": 1074' in idx_line
    states = [(e.etype, e.state) for e in app.events.query()]
    assert ("corrections_link", "connected") in states
    corr_srv.close(); obs_srv.close()
```

- [ ] **Step 2: 运行确认失败**

Run: `pytest tests/test_e2e.py -v`
Expected: FAIL（`rtk_monitor.main` 不存在）

- [ ] **Step 3: 实现 main.py**

```python
# src/rtk_monitor/main.py
"""Wire config -> collectors -> writers/reserver/events. Entry point of Plan 1.

CAN channel naming: "can0" opens SocketCAN; "virtual:<name>" opens a python-can
virtual bus (tests / replay without hardware).
"""
from __future__ import annotations

import asyncio
import sys

import can

from rtk_monitor.config import Config, load_config
from rtk_monitor.collectors.can import CanCollector
from rtk_monitor.collectors.reserve import LocalReserver
from rtk_monitor.collectors.tcp import TcpCollector
from rtk_monitor.parsers.rtcm import RtcmFramer
from rtk_monitor.storage.canlog import CandumpWriter
from rtk_monitor.storage.cleanup import cleanup_logs
from rtk_monitor.storage.events import EventStore
from rtk_monitor.storage.rawlog import RawLogWriter

_CLEANUP_INTERVAL_S = 3600.0


class App:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        cfg.data_root.mkdir(parents=True, exist_ok=True)
        self.events = EventStore(cfg.db_path)
        self.corr_reserver = LocalReserver()
        self.obs_reserver = LocalReserver()

        self._corr_log = RawLogWriter(cfg.data_root, "corr", ext="rtcm3")
        self._obs_log = RawLogWriter(cfg.data_root, "obs", ext="rtcm3")
        self._sol_log = RawLogWriter(cfg.data_root, "sol", ext="txt")
        self._corr_framer = RtcmFramer()
        self._obs_framer = RtcmFramer()

        self._collectors = [
            TcpCollector("corrections_link", cfg.corrections.host, cfg.corrections.port,
                         self._on_corr, self._on_event, listen=cfg.corrections.listen),
            TcpCollector("raw_obs_link", cfg.raw_obs.host, cfg.raw_obs.port,
                         self._on_obs, self._on_event, listen=cfg.raw_obs.listen),
            TcpCollector("gnss_solution_link", cfg.gnss_solution.host, cfg.gnss_solution.port,
                         self._on_sol, self._on_event, listen=cfg.gnss_solution.listen),
        ]
        channel = cfg.can_channel
        if channel.startswith("virtual:"):
            self._bus = can.Bus(interface="virtual", channel=channel.split(":", 1)[1])
            log_name = channel.replace(":", "_")
        else:
            self._bus = can.Bus(interface="socketcan", channel=channel)
            log_name = channel
        self._can_log = CandumpWriter(cfg.data_root, log_name)
        self._can_collector = CanCollector(self._bus, self._on_can)

    # --- stream callbacks: log first, then fan out -------------------------
    def _on_corr(self, data: bytes, t: float) -> None:
        for msg in self._corr_framer.feed(data):
            self._corr_log.append(msg.raw, msg.msg_type)
        self.corr_reserver.broadcast(data)

    def _on_obs(self, data: bytes, t: float) -> None:
        for msg in self._obs_framer.feed(data):
            self._obs_log.append(msg.raw, msg.msg_type)
        self.obs_reserver.broadcast(data)

    def _on_sol(self, data: bytes, t: float) -> None:
        self._sol_log.append(data)

    def _on_can(self, can_id: int, data: bytes, t: float) -> None:
        self._can_log.append(can_id, data, t)

    def _on_event(self, name: str, state: str, detail: str) -> None:
        import time
        self.events.record(time.time(), name, state, detail)

    # --- lifecycle ---------------------------------------------------------
    async def run_forever(self) -> None:
        await self.corr_reserver.start(self.cfg.reserve_corrections_port)
        await self.obs_reserver.start(self.cfg.reserve_raw_obs_port)
        tasks = [asyncio.create_task(c.run()) for c in self._collectors]
        tasks.append(asyncio.create_task(self._can_collector.run()))
        tasks.append(asyncio.create_task(self._cleanup_loop()))
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            for t in tasks:
                t.cancel()
            raise

    async def _cleanup_loop(self) -> None:
        while True:
            cleanup_logs(self.cfg.data_root, self.cfg.retention_days,
                         self.cfg.disk_watermark_pct)
            await asyncio.sleep(_CLEANUP_INTERVAL_S)

    async def shutdown(self) -> None:
        for w in (self._corr_log, self._obs_log, self._sol_log, self._can_log):
            w.close()
        await self.corr_reserver.stop()
        await self.obs_reserver.stop()
        self._bus.shutdown()
        self.events.close()


def build_app(cfg: Config) -> App:
    return App(cfg)


def main() -> None:
    cfg = load_config(sys.argv[1] if len(sys.argv) > 1 else "config.yaml")
    app = build_app(cfg)
    try:
        asyncio.run(app.run_forever())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: 运行确认通过**

Run: `pytest tests/test_e2e.py -v && pytest -q`
Expected: e2e PASS，全套测试 PASS

- [ ] **Step 5: 写 tools/replay_sources.py（数据源模拟器）**

```python
#!/usr/bin/env python3
# tools/replay_sources.py
"""Replay recorded streams as live sources for end-to-end testing without a vehicle.

Usage:
  python tools/replay_sources.py --candump can.log --can-channel virtual:e2e \
      --rtcm corr.rtcm3 --rtcm-port 6001 --speed 10

Serves each --rtcm/--text file on its TCP port (loops forever), and replays a
candump log onto a python-can channel, all paced by original timestamps / speed.
"""
from __future__ import annotations

import argparse
import asyncio
import re

import can

LINE_RE = re.compile(r"\((?P<t>[\d.]+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)")


async def serve_file(path: str, port: int, chunk: int, interval: float) -> None:
    data = open(path, "rb").read()

    async def handler(reader, writer):
        try:
            while True:
                for off in range(0, len(data), chunk):
                    writer.write(data[off:off + chunk])
                    await writer.drain()
                    await asyncio.sleep(interval)
        except (ConnectionResetError, BrokenPipeError):
            pass

    server = await asyncio.start_server(handler, "0.0.0.0", port)
    print(f"serving {path} on :{port}")
    async with server:
        await server.serve_forever()


async def replay_candump(path: str, channel: str, speed: float) -> None:
    if channel.startswith("virtual:"):
        bus = can.Bus(interface="virtual", channel=channel.split(":", 1)[1])
    else:
        bus = can.Bus(interface="socketcan", channel=channel)
    prev_t = None
    with open(path) as f:
        for line in f:
            m = LINE_RE.match(line)
            if not m:
                continue
            t = float(m["t"])
            if prev_t is not None:
                await asyncio.sleep(max(0.0, (t - prev_t) / speed))
            prev_t = t
            bus.send(can.Message(arbitration_id=int(m["id"], 16),
                                 data=bytes.fromhex(m["data"]), is_extended_id=False))
    bus.shutdown()
    print("candump replay finished")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candump")
    ap.add_argument("--can-channel", default="virtual:e2e")
    ap.add_argument("--rtcm", action="append", default=[],
                    help="FILE:PORT, may repeat (corrections, raw obs)")
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    async def amain():
        tasks = []
        for spec in args.rtcm:
            path, port = spec.rsplit(":", 1)
            tasks.append(serve_file(path, int(port), chunk=512, interval=0.1 / args.speed))
        if args.candump:
            tasks.append(replay_candump(args.candump, args.can_channel, args.speed))
        await asyncio.gather(*tasks)

    asyncio.run(amain())


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: 手工验证模拟器（用 8/27 真实录包）**

```bash
head -c 2000000 /home/steve/deploy_ws/issues/gnss/can_record/candump-2026-08-27_121555.log > /tmp/can_sample.log
python tools/replay_sources.py --candump /tmp/can_sample.log --can-channel virtual:e2e --speed 60
```
另开终端跑 `python -m rtk_monitor.main`（config 指向 virtual:e2e），确认 `<data_root>/YYYYMMDD/` 下 can 日志在增长且行格式与原 candump 一致。

- [ ] **Step 7: Commit**

```bash
git add src/rtk_monitor/main.py tools/replay_sources.py tests/test_e2e.py
git commit -m "feat: app assembly, replay simulator and end-to-end test"
```

---

## Self-Review 记录

- **Spec 覆盖**：§3.1 四路接入（Task 7/8/11）、sidecar 索引（Task 5）、§3.2 目录/清理/断线事件（Task 5/10/6/7）、§3.3 本地转发（Task 9）、§8 模拟器（Task 11）、§9 config 集中化（Task 1）。§3.1 中路 3 的"解析入 SQLite（历元表）"属于 Plan 2 历元表范围，本计划先落盘裸流并提供 parser（Task 3/4），入库在 Plan 2 与 solver 输出一起做——已在 Plan 2 待办中注明。
- **占位符扫描**：无 TBD/TODO；所有步骤含完整代码。
- **类型一致性**：`TcpCollector` 回调签名在 Task 7 定义、Task 11 按同签名消费；`RawLogWriter.append(data, msg_type)` 在 Task 5 定义、Task 8/11 一致；`EventStore.record(t, etype, state, detail)` 在 Task 6 定义、Task 11 `_on_event` 一致；`Config` 字段名与 Task 1 测试一致。
