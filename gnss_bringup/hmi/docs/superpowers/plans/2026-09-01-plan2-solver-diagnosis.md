# Plan 2: 解算与诊断 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** rtkrcv 独立 RTK 解算集成、诊断规则链与事件状态机、历元入库（rtkrcv/GPCHC/CAN 三源）、基站坐标监测、UDP 对外发布，外加 Plan 1 停放的三个修复项。

**Architecture:** rtkrcv 作为受监督的子进程（Plan 1 的 LocalReserver 喂数、TcpCollector 回读 solution 流）；解析器仍为纯函数；App 新增 1Hz 诊断循环，从各路最新状态组装 `DiagnosisInput`，跑纯函数规则链得 `Verdict`，驱动事件状态机与 UDP 发布器。所有新增后台任务走 Plan 1 的 `_supervise` 隔离。

**Tech Stack:** Python 3.11 asyncio（沿用 Plan 1 栈，无新增运行时依赖）；RTKLIB demo5 `rtkrcv`（外部二进制，测试用 fake 脚本替身）。

**Spec:** `docs/superpowers/specs/2026-08-31-rtk-monitor-design.md`（本计划实现 §4 全部、§2.1 solver/diagnosis/publisher/store 的历元部分、§7、§3.1 路 3/4 "解析入 SQLite"）

## Global Constraints

- Python >= 3.11；运行时第三方依赖仍仅 `pyyaml`、`python-can`——本计划不得新增依赖。
- 解析器不做任何 IO；诊断规则链是纯函数（输入 dataclass → 输出 Verdict）。
- 诊断阈值全部来自 config（spec §4.2 默认值），不散落硬编码。
- 结论文案用中文、与 spec §4.2 模板一致；代码与注释用英文。
- rtkrcv 二进制不进仓库、测试不依赖它——测试一律用 fake 脚本；真实二进制的构建与验证是文档化的集成步骤（Task 14）。
- 每个 commit 信息末尾带：`Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。
- 每个任务提交前跑全套测试（Plan 1 基线 41 个必须保持全绿）。

## 文件结构（新增/修改总览）

```
src/rtk_monitor/
├── config.py                 # 修改：新增 RtkrcvCfg / DiagnosisCfg / PublishCfg（可选段，带默认值）
├── parsers/
│   ├── gpchc.py              # 修改：LineFramer 缓冲上限（Plan 1 停放项）
│   ├── rtksol.py             # 新增：rtkrcv llh solution 行解析
│   └── rtkstat.py            # 新增：$SAT 状态行解析 + 周跳滑窗
├── solver/
│   ├── __init__.py           # 新增（空）
│   └── rtkrcv.py             # 新增：conf 生成 + 子进程生命周期管理
├── diagnosis/
│   ├── __init__.py           # 新增（空）
│   ├── rules.py              # 新增：DiagnosisInput/Verdict + diagnose() 纯函数规则链
│   ├── events.py             # 新增：EventMachine（open/close + 迟滞）
│   └── base_station.py       # 新增：1005 基线学习与偏移监测
├── storage/
│   ├── epochs.py             # 新增：EpochStore（epochs/base_station/kv 表）
│   └── events.py             # 修改：schema 迁移（level/code/t_close/lat/lon 列）+ close_event
├── publisher.py              # 新增：UdpPublisher（gnss_fix / gnss_event JSON Lines）
├── collectors/
│   ├── tcp.py                # 修改：监听模式断连事件仅在状态转换时发（Plan 1 停放项）
│   └── can.py                # 修改：看门狗连续超时后重开总线（Plan 1 停放项）
└── main.py                   # 修改：路 3/4 历元接线、solver/诊断循环/发布器装配
tests/fake_rtkrcv.py          # 新增：rtkrcv 替身（可执行）
scripts/build_rtkrcv.sh       # 新增：demo5 构建脚本（无测试依赖）
docs/integration-rtkrcv.md    # 新增：真机集成核对清单
```

---

### Task 1: 配置扩展（rtkrcv / diagnosis / publish 段）

**Files:**
- Modify: `src/rtk_monitor/config.py`
- Modify: `config.yaml.example`
- Test: `tests/test_config.py`（追加）

**Interfaces:**
- Produces: `Config` 新增字段 `rtkrcv: RtkrcvCfg`、`diagnosis: DiagnosisCfg`、`publish: PublishCfg`。三段在 yaml 中**可选**——缺省时用默认值，Plan 1 的旧 config 必须仍能加载。
  - `RtkrcvCfg(binary: str = "", sol_port: int = 15020, extra_args: tuple[str, ...] = ())`；`binary` 为空字符串表示禁用 rtkrcv。
  - `DiagnosisCfg(corr_gap_s=3.0, age_max_s=10.0, base_shift_m=0.1, min_sats=6, resid_max_m=2.0, low_el_deg=20.0, low_snr_dbhz=35.0, min_ratio=3.0, slip_max_per_30s=5, divergence_sigma=3.0, divergence_hold_s=5.0, close_hysteresis_s=10.0)`（全 float 除 min_sats/slip_max_per_30s 为 int）。
  - `PublishCfg(enabled: bool = False, host: str = "127.0.0.1", port: int = 15030)`。

- [ ] **Step 1: 写失败测试（追加到 tests/test_config.py）**

```python
def test_plan2_sections_defaults(tmp_path):
    # A Plan-1-era config with no rtkrcv/diagnosis/publish sections must still load.
    p = tmp_path / "old.yaml"
    p.write_text(EXAMPLE.read_text())  # example will gain the sections; strip them
    text = "\n".join(l for l in p.read_text().splitlines()
                     if not l.startswith(("rtkrcv", "diagnosis", "publish"))
                     and not l.startswith(("  binary", "  sol_port", "  enabled")))
    p.write_text(text)
    cfg = load_config(p)
    assert cfg.rtkrcv.binary == "" and cfg.rtkrcv.sol_port == 15020
    assert cfg.diagnosis.corr_gap_s == 3.0 and cfg.diagnosis.min_sats == 6
    assert cfg.diagnosis.close_hysteresis_s == 10.0
    assert cfg.publish.enabled is False and cfg.publish.port == 15030

def test_plan2_sections_explicit():
    cfg = load_config(EXAMPLE)
    assert cfg.rtkrcv.sol_port == 15020          # example carries the sections
    assert cfg.publish.host == "127.0.0.1"
```

- [ ] **Step 2: 运行确认失败** — `pytest tests/test_config.py -v`，FAIL（Config 无 rtkrcv 属性）

- [ ] **Step 3: 实现**

config.py 追加（import 区无新增）：

```python
@dataclass(frozen=True)
class RtkrcvCfg:
    binary: str = ""
    sol_port: int = 15020
    extra_args: tuple[str, ...] = ()


@dataclass(frozen=True)
class DiagnosisCfg:
    corr_gap_s: float = 3.0
    age_max_s: float = 10.0
    base_shift_m: float = 0.1
    min_sats: int = 6
    resid_max_m: float = 2.0
    low_el_deg: float = 20.0
    low_snr_dbhz: float = 35.0
    min_ratio: float = 3.0
    slip_max_per_30s: int = 5
    divergence_sigma: float = 3.0
    divergence_hold_s: float = 5.0
    close_hysteresis_s: float = 10.0


@dataclass(frozen=True)
class PublishCfg:
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 15030
```

`Config` 增加三个字段 `rtkrcv: RtkrcvCfg`、`diagnosis: DiagnosisCfg`、`publish: PublishCfg`；`load_config` 增加：

```python
    r = raw.get("rtkrcv") or {}
    d = raw.get("diagnosis") or {}
    p = raw.get("publish") or {}
    ...
        rtkrcv=RtkrcvCfg(binary=str(r.get("binary", "")),
                         sol_port=int(r.get("sol_port", 15020)),
                         extra_args=tuple(r.get("extra_args", []))),
        diagnosis=DiagnosisCfg(**{k: type(getattr(DiagnosisCfg, k))(v)
                                  for k, v in d.items()}),
        publish=PublishCfg(enabled=bool(p.get("enabled", False)),
                           host=str(p.get("host", "127.0.0.1")),
                           port=int(p.get("port", 15030))),
```

注意 `DiagnosisCfg(**{...})`：未知键应报错（dataclass 天然 TypeError），类型按字段默认值的类型转换。config.yaml.example 追加：

```yaml
rtkrcv:           # independent RTK solver (empty binary = disabled)
  binary: ""
  sol_port: 15020

diagnosis: {}     # thresholds; omit keys to use spec defaults

publish:          # UDP JSON Lines output (GLIM phase-2 interface)
  enabled: false
  host: 127.0.0.1
  port: 15030
```

- [ ] **Step 4: 运行确认通过** — `pytest tests/test_config.py -v` 全 PASS，随后全套 `pytest -q`（用 `.venv/bin/python -m pytest`）
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: config sections for rtkrcv, diagnosis thresholds and UDP publish"`

---

### Task 2: rtkrcv llh solution 行解析

**Files:**
- Create: `src/rtk_monitor/parsers/rtksol.py`
- Test: `tests/test_rtksol.py`

**Interfaces:**
- Produces: `parse_llh_solution(line: str) -> RtkSolution | None`（注释行/坏行返回 None）。`RtkSolution(t: float, lat: float, lon: float, alt: float, q: int, ns: int, sdn: float, sde: float, sdu: float, age: float, ratio: float)`。`t` 为解算历元的 Unix 秒（由 GPST 字符串按 UTC 解析；与 UTC 有 18s 闰秒差，本项目内一致使用即可——诊断只做时间差比较）。Q 语义（RTKLIB）：1=固定 2=浮点 4=DGPS 5=单点。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_rtksol.py
from rtk_monitor.parsers.rtksol import RtkSolution, parse_llh_solution

LINE = ("2026/08/27 04:15:55.400   44.501234567   90.287654321   617.1234"
        "   1  38   0.0110   0.0123   0.0322  -0.0001   0.0002   0.0003"
        "   0.80    2.5")


def test_parse_solution_line():
    s = parse_llh_solution(LINE)
    assert isinstance(s, RtkSolution)
    assert abs(s.lat - 44.501234567) < 1e-9 and abs(s.lon - 90.287654321) < 1e-9
    assert abs(s.alt - 617.1234) < 1e-6
    assert s.q == 1 and s.ns == 38
    assert abs(s.sdn - 0.0110) < 1e-6 and abs(s.sde - 0.0123) < 1e-6
    assert abs(s.age - 0.80) < 1e-6 and abs(s.ratio - 2.5) < 1e-6
    import datetime
    expect = datetime.datetime(2026, 8, 27, 4, 15, 55, 400000,
                               tzinfo=datetime.timezone.utc).timestamp()
    assert abs(s.t - expect) < 1e-3


def test_comment_and_garbage_return_none():
    assert parse_llh_solution("% GPST latitude ...") is None
    assert parse_llh_solution("") is None
    assert parse_llh_solution("2026/08/27 04:15:55.400 not a number") is None
```

- [ ] **Step 2: 运行确认失败** — `pytest tests/test_rtksol.py -v`，FAIL（模块不存在）
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/parsers/rtksol.py
"""Parse RTKLIB llh-format solution lines from rtkrcv's output stream.

Column order (out-outhead off): date time lat lon height Q ns sdn sde sdu
sdne sdeu sdun age ratio. Timestamps are GPST parsed as UTC (constant ~18 s
offset; only time differences are consumed downstream).
"""
from __future__ import annotations

import datetime
from dataclasses import dataclass


@dataclass(frozen=True)
class RtkSolution:
    t: float
    lat: float
    lon: float
    alt: float
    q: int          # RTKLIB: 1=fix 2=float 4=dgps 5=single
    ns: int
    sdn: float
    sde: float
    sdu: float
    age: float
    ratio: float


def parse_llh_solution(line: str) -> RtkSolution | None:
    line = line.strip()
    if not line or line.startswith("%"):
        return None
    f = line.split()
    if len(f) < 15:
        return None
    try:
        dt = datetime.datetime.strptime(f[0] + " " + f[1], "%Y/%m/%d %H:%M:%S.%f")
        return RtkSolution(
            t=dt.replace(tzinfo=datetime.timezone.utc).timestamp(),
            lat=float(f[2]), lon=float(f[3]), alt=float(f[4]),
            q=int(f[5]), ns=int(f[6]),
            sdn=float(f[7]), sde=float(f[8]), sdu=float(f[9]),
            age=float(f[13]), ratio=float(f[14]),
        )
    except ValueError:
        return None
```

- [ ] **Step 4: 运行确认通过** — `pytest tests/test_rtksol.py -v` 3 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: rtkrcv llh solution line parser"`

---

### Task 3: $SAT 状态行解析 + 周跳滑窗

**Files:**
- Create: `src/rtk_monitor/parsers/rtkstat.py`
- Test: `tests/test_rtkstat.py`

**Interfaces:**
- Produces: `parse_sat_line(line: str) -> SatStat | None`；`SatStat(tow: float, sat: str, az: float, el: float, resp: float, snr: float, valid: bool, slipc: int, rejc: int)`。`SlipWindow(window_s: float = 30.0)`，方法 `feed(t: float, sat: str, slipc: int) -> None`（按每星 slipc 计数器的**增量**入窗）、`count(now: float) -> int`（窗口内总周跳次数）。
- RTKLIB $SAT CSV 布局（solution status ver.2）：`$SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc`。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_rtkstat.py
from rtk_monitor.parsers.rtkstat import SatStat, SlipWindow, parse_sat_line

LINE = "$SAT,2372,113755.4,G12,1,231.5,18.2,2.31,0.012,1,34.5,1,0,120,0,3,1"


def test_parse_sat_line():
    s = parse_sat_line(LINE)
    assert isinstance(s, SatStat)
    assert s.sat == "G12" and abs(s.tow - 113755.4) < 1e-6
    assert abs(s.az - 231.5) < 1e-6 and abs(s.el - 18.2) < 1e-6
    assert abs(s.resp - 2.31) < 1e-6 and abs(s.snr - 34.5) < 1e-6
    assert s.valid is True and s.slipc == 3 and s.rejc == 1


def test_non_sat_lines_return_none():
    assert parse_sat_line("$POS,2372,113755.4,...") is None
    assert parse_sat_line("$SAT,bad") is None


def test_slip_window_counts_increments_only():
    w = SlipWindow(window_s=30.0)
    w.feed(100.0, "G12", 3)      # first sighting: baseline, no increment
    w.feed(101.0, "G12", 5)      # +2
    w.feed(102.0, "C08", 1)      # baseline
    w.feed(103.0, "C08", 2)      # +1
    assert w.count(now=110.0) == 3
    assert w.count(now=140.0) == 0     # both increments aged out (>30 s)
```

- [ ] **Step 2: 运行确认失败** — `pytest tests/test_rtkstat.py -v`，FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/parsers/rtkstat.py
"""Parse rtkrcv solution-status $SAT lines; track cycle-slip counts in a window."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SatStat:
    tow: float
    sat: str
    az: float
    el: float
    resp: float       # pseudorange residual (m)
    snr: float        # dBHz
    valid: bool
    slipc: int
    rejc: int


def parse_sat_line(line: str) -> SatStat | None:
    if not line.startswith("$SAT,"):
        return None
    p = line.strip().split(",")
    if len(p) < 17:
        return None
    try:
        return SatStat(tow=float(p[2]), sat=p[3], az=float(p[5]), el=float(p[6]),
                       resp=float(p[7]), snr=float(p[10]), valid=p[9] == "1",
                       slipc=int(p[15]), rejc=int(p[16]))
    except ValueError:
        return None


class SlipWindow:
    """Count cycle-slip increments across all satellites within a sliding window."""

    def __init__(self, window_s: float = 30.0) -> None:
        self._window = window_s
        self._last: dict[str, int] = {}
        self._hits: list[tuple[float, int]] = []   # (t, delta)

    def feed(self, t: float, sat: str, slipc: int) -> None:
        prev = self._last.get(sat)
        self._last[sat] = slipc
        if prev is not None and slipc > prev:
            self._hits.append((t, slipc - prev))

    def count(self, now: float) -> int:
        cutoff = now - self._window
        self._hits = [(t, d) for t, d in self._hits if t >= cutoff]
        return sum(d for _, d in self._hits)
```

- [ ] **Step 4: 运行确认通过** — 3 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: rtkrcv \\$SAT stat parser and cycle-slip window"`

---

### Task 4: EpochStore（epochs / base_station / kv 表）

**Files:**
- Create: `src/rtk_monitor/storage/epochs.py`
- Test: `tests/test_epochs.py`

**Interfaces:**
- Produces: `Epoch` frozen dataclass：`t: float, src: str`（"rtkrcv"|"gpchc"|"can"），可选 `q, sats: int|None`，`age, lat, lon, alt, sde, sdn, sdu, ratio, heading, speed: float|None`，`sats_json: str|None`。
  `EpochStore(db_path)`：`add(e: Epoch) -> int`、`latest(src: str) -> Epoch | None`、`query(src: str, t0: float, t1: float) -> list[Epoch]`、`kv_get(k) -> str|None`、`kv_set(k, v)`、`add_base(t, x, y, z)`、`base_history(since=0.0) -> list[tuple[t,x,y,z]]`、`close()`。
- 注意：`q` 的语义随 src 而异——src=rtkrcv 为 RTKLIB Q（1=固定），src=gpchc/can 为 610 卫星状态（4=固定）。消费方按 src 解读；文档注释写明。
- 与 EventStore 同一个 db 文件（Task 11 用同一 `cfg.db_path` 分别构造两个 store；SQLite 同进程多连接安全）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_epochs.py
from rtk_monitor.storage.epochs import Epoch, EpochStore


def test_add_latest_query(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    s.add(Epoch(t=100.0, src="rtkrcv", q=1, sats=38, lat=44.5, lon=90.28,
                alt=617.1, sdn=0.011, sde=0.012, sdu=0.032, age=0.8, ratio=2.5))
    s.add(Epoch(t=101.0, src="rtkrcv", q=2, sats=35, ratio=1.8))
    s.add(Epoch(t=100.5, src="can", q=4, sats=39, heading=174.2, speed=8.3))
    latest = s.latest("rtkrcv")
    assert latest.t == 101.0 and latest.q == 2 and abs(latest.ratio - 1.8) < 1e-9
    assert s.latest("can").heading == 174.2
    assert s.latest("gpchc") is None
    rows = s.query("rtkrcv", 99.0, 100.5)
    assert len(rows) == 1 and rows[0].sats == 38 and rows[0].lat == 44.5
    s.close()


def test_kv_and_base(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    assert s.kv_get("base_xyz") is None
    s.kv_set("base_xyz", "1,2,3")
    s.kv_set("base_xyz", "4,5,6")            # upsert
    assert s.kv_get("base_xyz") == "4,5,6"
    s.add_base(100.0, -2148744.1, 4426641.2, 4044655.9)
    s.add_base(200.0, -2148744.2, 4426641.2, 4044655.9)
    hist = s.base_history(since=150.0)
    assert len(hist) == 1 and hist[0][0] == 200.0
    s.close()


def test_persists_across_reopen(tmp_path):
    p = tmp_path / "e.db"
    s = EpochStore(p)
    s.add(Epoch(t=1.0, src="gpchc", q=4))
    s.close()
    assert EpochStore(p).latest("gpchc").t == 1.0
```

- [ ] **Step 2: 运行确认失败** — `pytest tests/test_epochs.py -v`，FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/storage/epochs.py
"""SQLite epoch summaries (1 Hz per source) plus base-station history and a KV table.

`q` semantics depend on `src`: for "rtkrcv" it is the RTKLIB quality flag
(1=fix 2=float 4=dgps 5=single); for "gpchc"/"can" it is the CGI-610
satellite-status nibble (4=RTK fixed with heading, 5=float, ...).
"""
from __future__ import annotations

import sqlite3
from dataclasses import dataclass, fields
from pathlib import Path

_SCHEMA = """CREATE TABLE IF NOT EXISTS epochs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL, src TEXT NOT NULL,
    q INTEGER, sats INTEGER, age REAL,
    lat REAL, lon REAL, alt REAL,
    sde REAL, sdn REAL, sdu REAL,
    ratio REAL, heading REAL, speed REAL, sats_json TEXT
);
CREATE INDEX IF NOT EXISTS idx_epochs_t ON epochs(t);
CREATE INDEX IF NOT EXISTS idx_epochs_src_t ON epochs(src, t);
CREATE TABLE IF NOT EXISTS kv (k TEXT PRIMARY KEY, v TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS base_station (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    t REAL NOT NULL, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_base_t ON base_station(t);"""

_COLS = ("t", "src", "q", "sats", "age", "lat", "lon", "alt",
         "sde", "sdn", "sdu", "ratio", "heading", "speed", "sats_json")


@dataclass(frozen=True)
class Epoch:
    t: float
    src: str
    q: int | None = None
    sats: int | None = None
    age: float | None = None
    lat: float | None = None
    lon: float | None = None
    alt: float | None = None
    sde: float | None = None
    sdn: float | None = None
    sdu: float | None = None
    ratio: float | None = None
    heading: float | None = None
    speed: float | None = None
    sats_json: str | None = None


class EpochStore:
    def __init__(self, db_path: str | Path) -> None:
        self._db = sqlite3.connect(db_path)
        self._db.executescript(_SCHEMA)
        self._db.commit()

    def add(self, e: Epoch) -> int:
        vals = [getattr(e, c) for c in _COLS]
        cur = self._db.execute(
            f"INSERT INTO epochs ({','.join(_COLS)}) VALUES ({','.join('?' * len(_COLS))})",
            vals)
        self._db.commit()
        return int(cur.lastrowid)

    def _row_to_epoch(self, row) -> Epoch:
        return Epoch(**dict(zip(_COLS, row)))

    def latest(self, src: str) -> Epoch | None:
        row = self._db.execute(
            f"SELECT {','.join(_COLS)} FROM epochs WHERE src=? ORDER BY t DESC LIMIT 1",
            (src,)).fetchone()
        return self._row_to_epoch(row) if row else None

    def query(self, src: str, t0: float, t1: float) -> list[Epoch]:
        rows = self._db.execute(
            f"SELECT {','.join(_COLS)} FROM epochs WHERE src=? AND t>=? AND t<=? ORDER BY t",
            (src, t0, t1)).fetchall()
        return [self._row_to_epoch(r) for r in rows]

    def kv_get(self, k: str) -> str | None:
        row = self._db.execute("SELECT v FROM kv WHERE k=?", (k,)).fetchone()
        return row[0] if row else None

    def kv_set(self, k: str, v: str) -> None:
        self._db.execute(
            "INSERT INTO kv (k, v) VALUES (?, ?) ON CONFLICT(k) DO UPDATE SET v=excluded.v",
            (k, v))
        self._db.commit()

    def add_base(self, t: float, x: float, y: float, z: float) -> None:
        self._db.execute("INSERT INTO base_station (t, x, y, z) VALUES (?, ?, ?, ?)",
                         (t, x, y, z))
        self._db.commit()

    def base_history(self, since: float = 0.0) -> list[tuple[float, float, float, float]]:
        return self._db.execute(
            "SELECT t, x, y, z FROM base_station WHERE t>=? ORDER BY t",
            (since,)).fetchall()

    def close(self) -> None:
        self._db.close()
```

- [ ] **Step 4: 运行确认通过** — 3 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: epoch store with base-station history and kv table"`

---

### Task 5: EventStore 扩展（迁移 + close_event）

**Files:**
- Modify: `src/rtk_monitor/storage/events.py`
- Test: `tests/test_events.py`（追加）

**Interfaces:**
- Produces（向后兼容，旧调用不变）：`record(t, etype, state, detail="", level=None, code=None, lat=None, lon=None) -> int`；新增 `close_event(event_id: int, t_close: float) -> None`；`EventRow` 增加字段 `level: str|None, code: str|None, t_close: float|None, lat: float|None, lon: float|None`；旧库文件自动 ALTER 迁移。

- [ ] **Step 1: 写失败测试（追加）**

```python
def test_extended_columns_and_close(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "diagnosis", "open", "差分中断 12s",
                   level="serious", code="corr_outage", lat=44.5, lon=90.28)
    s.close_event(rid, 130.0)
    row = s.query()[0]
    assert row.level == "serious" and row.code == "corr_outage"
    assert row.t_close == 130.0 and row.lat == 44.5


def test_migrates_old_schema(tmp_path):
    import sqlite3
    p = tmp_path / "old.db"
    db = sqlite3.connect(p)
    db.execute("CREATE TABLE events (id INTEGER PRIMARY KEY AUTOINCREMENT,"
               " t REAL NOT NULL, etype TEXT NOT NULL, state TEXT NOT NULL,"
               " detail TEXT NOT NULL DEFAULT '')")
    db.execute("INSERT INTO events (t, etype, state) VALUES (1.0, 'x', 'open')")
    db.commit(); db.close()
    s = EventStore(p)                      # must not raise; must migrate
    rows = s.query()
    assert rows[0].etype == "x" and rows[0].level is None
```

- [ ] **Step 2: 运行确认失败** — FAIL（record 无 level 参数）
- [ ] **Step 3: 实现**

events.py 修改：`EventRow` 增加 `level: str | None = None`、`code: str | None = None`、`t_close: float | None = None`、`lat: float | None = None`、`lon: float | None = None`；`__init__` 建表后调用迁移：

```python
_EXTRA_COLS = (("level", "TEXT"), ("code", "TEXT"), ("t_close", "REAL"),
               ("lat", "REAL"), ("lon", "REAL"))

    def _migrate(self) -> None:
        cols = {r[1] for r in self._db.execute("PRAGMA table_info(events)")}
        for col, typ in _EXTRA_COLS:
            if col not in cols:
                self._db.execute(f"ALTER TABLE events ADD COLUMN {col} {typ}")
        self._db.commit()
```

`record` 签名扩展并写入新列；`query` 的 SELECT 改为 `id, t, etype, state, detail, level, code, t_close, lat, lon`；新增：

```python
    def close_event(self, event_id: int, t_close: float) -> None:
        self._db.execute("UPDATE events SET state='closed', t_close=? WHERE id=?",
                         (t_close, event_id))
        self._db.commit()
```

- [ ] **Step 4: 运行确认通过** — 追加 2 PASS，原有 events 测试不破 + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: event store diagnosis columns with auto-migration and close_event"`

---

### Task 6: 诊断规则链（纯函数）

**Files:**
- Create: `src/rtk_monitor/diagnosis/__init__.py`（空）
- Create: `src/rtk_monitor/diagnosis/rules.py`
- Test: `tests/test_rules.py`

**Interfaces:**
- Consumes: `DiagnosisCfg`（Task 1）、`RtkSolution`（Task 2）、`SatStat`（Task 3）。
- Produces: `DiagnosisInput` dataclass（字段见实现）；`Verdict(level: str, code: str, message: str)`，level ∈ {"ok","info","warning","serious","critical"}；`diagnose(inp: DiagnosisInput, cfg: DiagnosisCfg) -> Verdict`。规则优先级与文案按 spec §4.2。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_rules.py
from rtk_monitor.config import DiagnosisCfg
from rtk_monitor.diagnosis.rules import DiagnosisInput, Verdict, diagnose
from rtk_monitor.parsers.rtksol import RtkSolution
from rtk_monitor.parsers.rtkstat import SatStat

CFG = DiagnosisCfg()


def _sol(q=1, ns=38, ratio=25.0, age=0.8, sdn=0.011, sde=0.012):
    return RtkSolution(t=1000.0, lat=44.5, lon=90.28, alt=617.0, q=q, ns=ns,
                       sdn=sdn, sde=sde, sdu=0.03, age=age, ratio=ratio)


def _inp(**kw):
    base = dict(now=1000.0, corr_last_t=999.5, corr_age=0.8, base_offset_m=0.0,
                sol=_sol(), sol_t=999.8, sats=[], slip_count_30s=0,
                divergence_m=None, divergence_since=None)
    base.update(kw)
    return DiagnosisInput(**base)


def test_all_good_is_fixed():
    v = diagnose(_inp(), CFG)
    assert v.code == "rtk_fixed" and v.level == "ok"


def test_rule1_corr_outage_wins_over_everything():
    v = diagnose(_inp(corr_last_t=990.0, sol=_sol(q=2, ratio=1.5)), CFG)
    assert v.code == "corr_outage" and v.level == "serious"
    assert "差分中断 10s" in v.message


def test_rule1_age_overrun():
    v = diagnose(_inp(corr_age=15.0), CFG)
    assert v.code == "corr_outage"


def test_rule2_base_shift():
    v = diagnose(_inp(base_offset_m=0.8), CFG)
    assert v.code == "base_shift" and v.level == "critical"
    assert "0.80" in v.message


def test_rule3_low_sats():
    v = diagnose(_inp(sol=_sol(ns=4)), CFG)
    assert v.code == "low_sats" and "4" in v.message


def test_rule4_multipath():
    sats = [SatStat(0, "C08", 90, 15, 3.5, 30, True, 0, 0),
            SatStat(0, "G17", 120, 12, 2.8, 33, True, 0, 0)]
    v = diagnose(_inp(sats=sats, sol=_sol(q=2, ratio=1.5)), CFG)
    assert v.code == "multipath" and "C08" in v.message and "G17" in v.message


def test_rule5_float_low_ratio():
    v = diagnose(_inp(sol=_sol(q=2, ratio=1.8)), CFG)
    assert v.code == "ambiguity" and "1.8" in v.message


def test_rule6_cycle_slips():
    v = diagnose(_inp(slip_count_30s=9), CFG)
    assert v.code == "cycle_slip"


def test_rule7_divergence_needs_hold():
    v = diagnose(_inp(divergence_m=0.5, divergence_since=998.0), CFG)
    assert v.code == "rtk_fixed"          # only 2 s, hold is 5 s
    v = diagnose(_inp(divergence_m=0.5, divergence_since=990.0), CFG)
    assert v.code == "device_divergence" and "0.50" in v.message


def test_no_data_at_all():
    v = diagnose(_inp(sol=None, sol_t=None, corr_last_t=None, corr_age=None), CFG)
    assert v.code == "no_data" and v.level == "warning"
```

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/diagnosis/rules.py
"""Pure diagnosis rule chain (spec §4.2): first matching rule wins."""
from __future__ import annotations

from dataclasses import dataclass, field

from rtk_monitor.config import DiagnosisCfg
from rtk_monitor.parsers.rtksol import RtkSolution
from rtk_monitor.parsers.rtkstat import SatStat


@dataclass(frozen=True)
class DiagnosisInput:
    now: float
    corr_last_t: float | None          # host time of last RTCM message
    corr_age: float | None             # differential age from solution (s)
    base_offset_m: float | None        # 1005 offset vs learned baseline
    sol: RtkSolution | None            # latest independent solution
    sol_t: float | None                # host time it arrived
    sats: list[SatStat] = field(default_factory=list)
    slip_count_30s: int = 0
    divergence_m: float | None = None      # |610 fused - rtkrcv| horizontal
    divergence_since: float | None = None  # host time divergence first exceeded


@dataclass(frozen=True)
class Verdict:
    level: str    # ok | info | warning | serious | critical
    code: str
    message: str


def diagnose(inp: DiagnosisInput, cfg: DiagnosisCfg) -> Verdict:
    if inp.sol is None and inp.corr_last_t is None:
        return Verdict("warning", "no_data", "无数据——检查采集链路与设备连接")

    # Rule 1: correction outage / age overrun
    gap = inp.now - inp.corr_last_t if inp.corr_last_t is not None else None
    if (gap is not None and gap > cfg.corr_gap_s) or \
       (inp.corr_age is not None and inp.corr_age > cfg.age_max_s):
        n = int(gap if gap is not None and gap > cfg.corr_gap_s else inp.corr_age)
        return Verdict("serious", "corr_outage",
                       f"差分中断 {n}s——5G 链路或平台转发问题")

    # Rule 2: base station coordinate shift
    if inp.base_offset_m is not None and inp.base_offset_m > cfg.base_shift_m:
        return Verdict("critical", "base_shift",
                       f"⚠ 基站坐标变动 {inp.base_offset_m:.2f}m——全矿定位将整体平移")

    # Rule 3: too few satellites
    if inp.sol is not None and inp.sol.ns < cfg.min_sats:
        return Verdict("serious", "low_sats",
                       f"卫星数不足（{inp.sol.ns} 颗）——高帮/坑底遮挡")

    # Rule 4: multipath suspects
    bad = [s for s in inp.sats
           if s.resp > cfg.resid_max_m and (s.el < cfg.low_el_deg or s.snr < cfg.low_snr_dbhz)]
    if len(bad) >= 2:
        names = "、".join(s.sat for s in bad[:4])
        return Verdict("warning", "multipath", f"{names} 残差异常——疑似多路径")

    # Rule 5: ambiguity not fixed
    if inp.sol is not None and inp.sol.q == 2 and inp.sol.ratio < cfg.min_ratio:
        return Verdict("warning", "ambiguity",
                       f"模糊度无法固定（ratio={inp.sol.ratio:.1f}）——遮挡过渡区常见")

    # Rule 6: frequent cycle slips
    if inp.slip_count_30s > cfg.slip_max_per_30s:
        return Verdict("warning", "cycle_slip",
                       "载波频繁失锁——动态遮挡或天线/馈线问题")

    # Rule 7: 610 output diverges from independent solution
    if (inp.divergence_m is not None and inp.divergence_since is not None
            and inp.sol is not None
            and inp.divergence_m > cfg.divergence_sigma * max(
                1e-3, (inp.sol.sdn ** 2 + inp.sol.sde ** 2) ** 0.5)
            and inp.now - inp.divergence_since >= cfg.divergence_hold_s):
        return Verdict("serious", "device_divergence",
                       f"610 输出与独立解算偏差 {inp.divergence_m:.2f}m——疑似 610 融合问题")

    if inp.sol is not None and inp.sol.q != 1:
        return Verdict("info", "not_fixed", f"非固定解（Q={inp.sol.q}）")
    return Verdict("ok", "rtk_fixed", "RTK 固定")
```

- [ ] **Step 4: 运行确认通过** — 11 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: pure diagnosis rule chain per spec 4.2"`

---

### Task 7: 事件状态机

**Files:**
- Create: `src/rtk_monitor/diagnosis/events.py`
- Test: `tests/test_event_machine.py`

**Interfaces:**
- Consumes: `Verdict`（Task 6）、`EventStore.record/close_event`（Task 5）。
- Produces: `EventMachine(store: EventStore, close_hysteresis_s: float = 10.0, on_transition=None)`，方法 `update(t: float, verdict: Verdict, lat: float | None = None, lon: float | None = None) -> None`。语义：非 ok 结论首次出现 → open 事件（etype="diagnosis"）；结论 code 变化 → 关旧开新；结论回到 ok/info 且持续 ≥ 迟滞 → close。`on_transition(kind: str, verdict: Verdict, t: float)` 在 "open"/"close" 时回调（发布器用）。level 为 info 的结论不开事件（只有 warning 及以上开）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_event_machine.py
from rtk_monitor.diagnosis.events import EventMachine
from rtk_monitor.diagnosis.rules import Verdict
from rtk_monitor.storage.events import EventStore

OK = Verdict("ok", "rtk_fixed", "RTK 固定")
OUT = Verdict("serious", "corr_outage", "差分中断 5s")
FLOAT_ = Verdict("warning", "ambiguity", "模糊度无法固定")
INFO = Verdict("info", "not_fixed", "非固定解（Q=2）")


def _machine(tmp_path, transitions):
    store = EventStore(tmp_path / "e.db")
    m = EventMachine(store, close_hysteresis_s=10.0,
                     on_transition=lambda kind, v, t: transitions.append((kind, v.code, t)))
    return store, m


def test_open_close_with_hysteresis(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, OK)
    m.update(101.0, OUT, lat=44.5, lon=90.2)
    m.update(105.0, OUT)
    m.update(106.0, OK)               # recovery starts
    m.update(110.0, OK)               # only 4 s ok — still open
    rows = store.query()
    assert len(rows) == 1 and rows[0].state == "open" and rows[0].code == "corr_outage"
    m.update(117.0, OK)               # 11 s ok — close
    rows = store.query()
    assert rows[0].state == "closed" and rows[0].t_close == 117.0
    assert tr == [("open", "corr_outage", 101.0), ("close", "corr_outage", 117.0)]


def test_code_change_closes_and_opens(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, OUT)
    m.update(101.0, FLOAT_)
    rows = store.query()
    assert [r.code for r in rows] == ["corr_outage", "ambiguity"]
    assert rows[0].state == "closed" and rows[1].state == "open"


def test_relapse_resets_hysteresis(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, OUT)
    m.update(101.0, OK)
    m.update(105.0, OUT)              # relapse before hysteresis elapsed
    m.update(120.0, OK)
    m.update(131.0, OK)               # 11 s after last ok start → close
    rows = store.query()
    assert len(rows) == 1 and rows[0].state == "closed"


def test_info_does_not_open(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, INFO)
    assert store.query() == [] and tr == []
```

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/diagnosis/events.py
"""Open/close diagnosis events with close hysteresis (spec §4.3)."""
from __future__ import annotations

from typing import Callable

from rtk_monitor.diagnosis.rules import Verdict
from rtk_monitor.storage.events import EventStore

_OPEN_LEVELS = {"warning", "serious", "critical"}
OnTransition = Callable[[str, Verdict, float], None]


class EventMachine:
    def __init__(self, store: EventStore, close_hysteresis_s: float = 10.0,
                 on_transition: OnTransition | None = None) -> None:
        self._store = store
        self._hyst = close_hysteresis_s
        self._cb = on_transition
        self._open_id: int | None = None
        self._open_code: str | None = None
        self._open_verdict: Verdict | None = None
        self._ok_since: float | None = None

    def update(self, t: float, verdict: Verdict,
               lat: float | None = None, lon: float | None = None) -> None:
        active = verdict.level in _OPEN_LEVELS
        if active:
            self._ok_since = None
            if self._open_code == verdict.code:
                return
            if self._open_id is not None:            # different condition: close old
                self._close(t)
            self._open_id = self._store.record(
                t, "diagnosis", "open", verdict.message,
                level=verdict.level, code=verdict.code, lat=lat, lon=lon)
            self._open_code = verdict.code
            self._open_verdict = verdict
            if self._cb:
                self._cb("open", verdict, t)
            return
        # ok/info: close after hysteresis
        if self._open_id is None:
            return
        if self._ok_since is None:
            self._ok_since = t
            return
        if t - self._ok_since >= self._hyst:
            self._close(t)

    def _close(self, t: float) -> None:
        assert self._open_id is not None and self._open_verdict is not None
        self._store.close_event(self._open_id, t)
        if self._cb:
            self._cb("close", self._open_verdict, t)
        self._open_id = self._open_code = self._open_verdict = None
        self._ok_since = None
```

- [ ] **Step 4: 运行确认通过** — 4 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: diagnosis event machine with close hysteresis"`

---

### Task 8: 基站坐标监测

**Files:**
- Create: `src/rtk_monitor/diagnosis/base_station.py`
- Test: `tests/test_base_station.py`

**Interfaces:**
- Consumes: `EpochStore.kv_get/kv_set/add_base`（Task 4）。
- Produces: `BaseStationMonitor(store: EpochStore, warmup_s: float = 600.0)`，方法 `feed(t: float, x: float, y: float, z: float) -> float | None`——基线未定时返回 None 并累积样本（首样本起 warmup_s 后取各轴中位数持久化到 kv "base_xyz"）；基线已定时返回三维偏移距离（m）。坐标变化 > 1 mm 或首次时写入 base_station 历史表。`reset(t, x, y, z)` 供人工确认后一键更新基线（界面用，Plan 3 接线）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_base_station.py
from rtk_monitor.diagnosis.base_station import BaseStationMonitor
from rtk_monitor.storage.epochs import EpochStore

XYZ = (-2148744.1000, 4426641.2000, 4044655.9000)


def test_learns_baseline_then_reports_offset(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=100.0)
    assert m.feed(0.0, *XYZ) is None
    assert m.feed(50.0, XYZ[0] + 0.001, XYZ[1], XYZ[2]) is None    # still warming up
    off = m.feed(101.0, *XYZ)                                       # warmup elapsed
    assert off is not None and off < 0.002                          # ~median
    assert store.kv_get("base_xyz") is not None
    off = m.feed(102.0, XYZ[0] + 0.5, XYZ[1], XYZ[2])
    assert abs(off - 0.5) < 0.01


def test_baseline_persists_across_restart(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=1.0)
    m.feed(0.0, *XYZ)
    m.feed(2.0, *XYZ)                       # baseline set
    store2 = EpochStore(tmp_path / "e.db")
    m2 = BaseStationMonitor(store2, warmup_s=1.0)
    assert m2.feed(10.0, *XYZ) is not None  # no re-warmup


def test_history_records_changes_only(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=1.0)
    m.feed(0.0, *XYZ)
    m.feed(2.0, *XYZ)          # same coords: history has the first sighting only
    m.feed(3.0, XYZ[0] + 0.5, XYZ[1], XYZ[2])
    hist = store.base_history()
    assert len(hist) == 2


def test_reset_updates_baseline(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=1.0)
    m.feed(0.0, *XYZ); m.feed(2.0, *XYZ)
    m.reset(5.0, XYZ[0] + 0.5, XYZ[1], XYZ[2])
    assert m.feed(6.0, XYZ[0] + 0.5, XYZ[1], XYZ[2]) < 0.01
```

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/diagnosis/base_station.py
"""Learn the base station's ECEF baseline from 1005 messages; report offsets."""
from __future__ import annotations

import math
import statistics

from rtk_monitor.storage.epochs import EpochStore

_KV_KEY = "base_xyz"


class BaseStationMonitor:
    def __init__(self, store: EpochStore, warmup_s: float = 600.0) -> None:
        self._store = store
        self._warmup = warmup_s
        self._samples: list[tuple[float, float, float, float]] = []
        self._last_hist: tuple[float, float, float] | None = None
        self._baseline: tuple[float, float, float] | None = None
        stored = store.kv_get(_KV_KEY)
        if stored:
            x, y, z = (float(v) for v in stored.split(","))
            self._baseline = (x, y, z)

    def feed(self, t: float, x: float, y: float, z: float) -> float | None:
        if self._last_hist is None or any(
                abs(a - b) > 1e-3 for a, b in zip((x, y, z), self._last_hist)):
            self._store.add_base(t, x, y, z)
            self._last_hist = (x, y, z)
        if self._baseline is None:
            self._samples.append((t, x, y, z))
            if t - self._samples[0][0] >= self._warmup:
                bx = statistics.median(s[1] for s in self._samples)
                by = statistics.median(s[2] for s in self._samples)
                bz = statistics.median(s[3] for s in self._samples)
                self._set_baseline(bx, by, bz)
            else:
                return None
        assert self._baseline is not None
        bx, by, bz = self._baseline
        return math.dist((x, y, z), (bx, by, bz))

    def reset(self, t: float, x: float, y: float, z: float) -> None:
        """Operator-confirmed baseline update (surfaced in the UI in Plan 3)."""
        self._set_baseline(x, y, z)
        self._store.add_base(t, x, y, z)
        self._last_hist = (x, y, z)

    def _set_baseline(self, x: float, y: float, z: float) -> None:
        self._baseline = (x, y, z)
        self._store.kv_set(_KV_KEY, f"{x:.4f},{y:.4f},{z:.4f}")
        self._samples.clear()
```

- [ ] **Step 4: 运行确认通过** — 4 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: base-station baseline monitor over RTCM 1005"`

---

### Task 9: UDP 发布器

**Files:**
- Create: `src/rtk_monitor/publisher.py`
- Test: `tests/test_publisher.py`

**Interfaces:**
- Produces: `UdpPublisher(host: str, port: int)`：`async start()`、`publish_fix(sol: RtkSolution, heading: float | None = None)`、`publish_event(kind: str, verdict: Verdict, t: float)`、`async stop()`。消息为 spec §7 的 JSON Lines（每条一行 `\n` 结尾，`ver:1`）。发送失败静默忽略（UDP 尽力而为）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_publisher.py
import asyncio
import json
import socket

from rtk_monitor.diagnosis.rules import Verdict
from rtk_monitor.parsers.rtksol import RtkSolution
from rtk_monitor.publisher import UdpPublisher


async def test_fix_and_event_lines():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0)); rx.setblocking(False)
    port = rx.getsockname()[1]
    p = UdpPublisher("127.0.0.1", port)
    await p.start()
    sol = RtkSolution(t=1000.0, lat=44.5, lon=90.28, alt=617.0, q=1, ns=38,
                      sdn=0.011, sde=0.012, sdu=0.032, age=0.8, ratio=25.0)
    p.publish_fix(sol, heading=174.2)
    p.publish_event("open", Verdict("serious", "corr_outage", "差分中断 5s"), 1000.5)
    await asyncio.sleep(0.05)
    loop = asyncio.get_running_loop()
    msgs = []
    for _ in range(2):
        data = await asyncio.wait_for(loop.sock_recv(rx, 4096), 1.0)
        msgs.append(json.loads(data.decode().strip()))
    fix = next(m for m in msgs if m["type"] == "gnss_fix")
    ev = next(m for m in msgs if m["type"] == "gnss_event")
    assert fix["ver"] == 1 and fix["q"] == 1 and fix["lat"] == 44.5
    assert fix["sigma_e"] == 0.012 and fix["heading"] == 174.2
    assert fix["source"] == "rtkrcv"
    assert ev["event"] == "corr_outage" and ev["state"] == "open"
    await p.stop(); rx.close()


async def test_publish_without_start_is_noop():
    p = UdpPublisher("127.0.0.1", 9)      # never started
    p.publish_event("open", Verdict("warning", "x", "y"), 1.0)  # must not raise
```

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/publisher.py
"""Best-effort UDP JSON Lines publisher (spec §7, GLIM phase-2 interface)."""
from __future__ import annotations

import asyncio
import json

from rtk_monitor.diagnosis.rules import Verdict
from rtk_monitor.parsers.rtksol import RtkSolution


class UdpPublisher:
    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_running_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            asyncio.DatagramProtocol, remote_addr=(self._host, self._port))

    def _send(self, obj: dict) -> None:
        if self._transport is None or self._transport.is_closing():
            return
        try:
            self._transport.sendto((json.dumps(obj, ensure_ascii=False) + "\n").encode())
        except OSError:
            pass

    def publish_fix(self, sol: RtkSolution, heading: float | None = None) -> None:
        self._send({"type": "gnss_fix", "ver": 1, "gps_time": sol.t,
                    "lat": sol.lat, "lon": sol.lon, "alt": sol.alt,
                    "q": sol.q, "sigma_e": sol.sde, "sigma_n": sol.sdn,
                    "sigma_u": sol.sdu, "heading": heading, "source": "rtkrcv"})

    def publish_event(self, kind: str, verdict: Verdict, t: float) -> None:
        self._send({"type": "gnss_event", "ver": 1, "gps_time": t,
                    "event": verdict.code, "state": kind, "detail": verdict.message})

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None
```

- [ ] **Step 4: 运行确认通过** — 2 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: UDP JSON Lines publisher for fixes and events"`

---

### Task 10: rtkrcv 子进程管理

**Files:**
- Create: `src/rtk_monitor/solver/__init__.py`（空）
- Create: `src/rtk_monitor/solver/rtkrcv.py`
- Test: `tests/test_rtkrcv_manager.py`

**Interfaces:**
- Consumes: `RtkrcvCfg`（Task 1）。
- Produces: `RtkrcvManager(binary: str, run_dir: Path, corr_port: int, obs_port: int, sol_port: int, extra_args: tuple = (), restart_delay: float = 5.0, on_event=None)`：`write_conf() -> Path`（生成 rtkrcv.conf）、`async run()`（拉起子进程；退出则 on_event + 延迟重启；取消时 terminate→5s 后 kill）。子进程环境注入 `RTKRCV_SOL_PORT=<sol_port>`（真实 rtkrcv 忽略之，测试替身用它选端口）。
- conf 模板的键名以 RTKLIB demo5 为准，**真机核对属于 Task 14 的集成清单**（本任务只保证生成与进程管理正确）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_rtkrcv_manager.py
import asyncio
import os
import stat

from rtk_monitor.solver.rtkrcv import RtkrcvManager


def _fake_binary(tmp_path, body: str) -> str:
    p = tmp_path / "fake_rtkrcv"
    p.write_text("#!/bin/sh\n" + body)
    p.chmod(p.stat().st_mode | stat.S_IEXEC)
    return str(p)


def test_write_conf(tmp_path):
    m = RtkrcvManager("/bin/true", tmp_path, corr_port=15010, obs_port=15011,
                      sol_port=15020)
    conf = m.write_conf()
    text = conf.read_text()
    assert "inpstr1-path =127.0.0.1:15011" in text      # rover = raw obs
    assert "inpstr2-path =127.0.0.1:15010" in text      # base = corrections
    assert "outstr1-path =:15020" in text
    assert "pos1-posmode =kinematic" in text


async def test_restarts_after_exit_and_terminates_on_cancel(tmp_path):
    marker = tmp_path / "runs"
    binary = _fake_binary(
        tmp_path, f'echo run >> "{marker}"\nsleep 0.1\n')
    events = []
    m = RtkrcvManager(binary, tmp_path, 15010, 15011, 15020,
                      restart_delay=0.05,
                      on_event=lambda n, s, d: events.append((n, s)))
    task = asyncio.create_task(m.run())
    await asyncio.sleep(0.6)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert marker.read_text().count("run") >= 2          # restarted at least once
    assert ("rtkrcv", "disconnected") in events


async def test_sol_port_in_env(tmp_path):
    out = tmp_path / "env"
    binary = _fake_binary(tmp_path, f'echo "$RTKRCV_SOL_PORT" > "{out}"\nsleep 5\n')
    m = RtkrcvManager(binary, tmp_path, 15010, 15011, 15021, restart_delay=9)
    task = asyncio.create_task(m.run())
    await asyncio.sleep(0.3)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert out.read_text().strip() == "15021"
```

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/solver/rtkrcv.py
"""Generate rtkrcv.conf and supervise the rtkrcv subprocess.

Config keys target RTKLIB demo5; verifying exact key names against the real
binary is an integration step (docs/integration-rtkrcv.md), not a unit test.
"""
from __future__ import annotations

import asyncio
import logging
import os
from pathlib import Path
from typing import Callable

_logger = logging.getLogger(__name__)

_CONF_TEMPLATE = """\
inpstr1-type =tcpcli
inpstr1-path =127.0.0.1:{obs_port}
inpstr1-format =rtcm3
inpstr2-type =tcpcli
inpstr2-path =127.0.0.1:{corr_port}
inpstr2-format =rtcm3
outstr1-type =tcpsvr
outstr1-path =:{sol_port}
outstr1-format =llh
pos1-posmode =kinematic
pos1-navsys =63
pos1-elmask =10
pos2-armode =continuous
out-solformat =llh
out-outhead =off
out-timesys =gpst
misc-svrcycle =10
"""


class RtkrcvManager:
    def __init__(self, binary: str, run_dir: Path, corr_port: int, obs_port: int,
                 sol_port: int, extra_args: tuple[str, ...] = (),
                 restart_delay: float = 5.0,
                 on_event: Callable[[str, str, str], None] | None = None) -> None:
        self._binary = binary
        self._run_dir = Path(run_dir)
        self._corr_port = corr_port
        self._obs_port = obs_port
        self._sol_port = sol_port
        self._extra = extra_args
        self._delay = restart_delay
        self._on_event = on_event

    def write_conf(self) -> Path:
        self._run_dir.mkdir(parents=True, exist_ok=True)
        conf = self._run_dir / "rtkrcv.conf"
        conf.write_text(_CONF_TEMPLATE.format(
            obs_port=self._obs_port, corr_port=self._corr_port,
            sol_port=self._sol_port))
        return conf

    async def run(self) -> None:
        conf = self.write_conf()
        env = dict(os.environ, RTKRCV_SOL_PORT=str(self._sol_port))
        while True:
            proc = None
            try:
                # extra_args precede the fixed flags: lets tests spawn
                # "python fake_rtkrcv.py ..." (Task 13); empty for the real binary
                proc = await asyncio.create_subprocess_exec(
                    self._binary, *self._extra, "-s", "-nc", "-o", str(conf),
                    cwd=self._run_dir, env=env,
                    stdout=asyncio.subprocess.DEVNULL,
                    stderr=asyncio.subprocess.DEVNULL)
                if self._on_event:
                    self._on_event("rtkrcv", "connected", f"pid {proc.pid}")
                rc = await proc.wait()
                if self._on_event:
                    self._on_event("rtkrcv", "disconnected", f"exit code {rc}")
            except asyncio.CancelledError:
                if proc is not None and proc.returncode is None:
                    proc.terminate()
                    try:
                        await asyncio.wait_for(proc.wait(), 5.0)
                    except asyncio.TimeoutError:
                        proc.kill()
                raise
            except Exception:
                _logger.exception("rtkrcv spawn failed")
                if self._on_event:
                    self._on_event("rtkrcv", "disconnected", "spawn failed")
            await asyncio.sleep(self._delay)
```

- [ ] **Step 4: 运行确认通过** — 3 PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: rtkrcv conf generation and supervised subprocess manager"`

---

### Task 11: Plan 1 停放项修复（监听事件转换化 / CAN 总线重开 / LineFramer 上限）

**Files:**
- Modify: `src/rtk_monitor/collectors/tcp.py`
- Modify: `src/rtk_monitor/collectors/can.py`
- Modify: `src/rtk_monitor/parsers/gpchc.py`
- Test: `tests/test_tcp_collector.py`、`tests/test_can_collector.py`、`tests/test_gpchc.py`（各追加）

**Interfaces:**
- Consumes: 现有 TcpCollector/CanCollector/LineFramer。
- Produces: 行为修复，公开签名不变，除 (b)：`CanCollector(bus, on_frame, on_event=None, data_timeout=2.0, bus_factory=None, reopen_after: int = 5)` 新增两参（默认不重开，向后兼容）。

三个修复：
(a) tcp.py 监听模式 handler 的 "disconnected" 也走 `_last_state` 转换判断（与客户端模式一致），连接建立时置 "connected"；
(b) can.py 看门狗连续 `reopen_after` 次超时后：`bus.shutdown()` → `bus_factory()` 造新 bus → 事件 `("can_link","reopened",...)`，计数清零；`bus_factory=None` 时不重开（仅事件，现行为）；
(c) gpchc.py `LineFramer(max_buf: int = 65536)`：缓冲超限时丢弃缓冲并计数 `self.overflows += 1`（对端永不发换行时防内存增长）。

- [ ] **Step 1: 写失败测试（三处各追加一个）**

```python
# tests/test_tcp_collector.py 追加
async def test_listen_mode_disconnect_event_only_on_transition():
    events = []
    c = TcpCollector("sol", "127.0.0.1", 0, on_data=lambda d, t: None,
                     on_event=lambda n, s, det: events.append(s), listen=True)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.05)
    for _ in range(3):                       # flapping peer
        _, w = await asyncio.open_connection("127.0.0.1", c.bound_port)
        w.close()
        await asyncio.sleep(0.05)
    task.cancel()
    # one connected+disconnected pair per actual transition, not per flap beyond first
    assert events.count("disconnected") <= events.count("connected")
```

```python
# tests/test_can_collector.py 追加
async def test_bus_reopened_after_consecutive_timeouts():
    made = []
    def factory():
        bus = can.Bus(interface="virtual", channel="reopen-test")
        made.append(bus)
        return bus
    events = []
    first = factory()
    c = CanCollector(first, on_frame=lambda *a: None,
                     on_event=lambda n, s, d: events.append(s),
                     data_timeout=0.05, bus_factory=factory, reopen_after=2)
    task = asyncio.create_task(c.run())
    await asyncio.sleep(0.5)                  # several timeouts → at least one reopen
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert "reopened" in events and len(made) >= 2
    for b in made:
        try: b.shutdown()
        except Exception: pass
```

```python
# tests/test_gpchc.py 追加
def test_line_framer_bounds_buffer():
    f = LineFramer(max_buf=64)
    f.feed(b"x" * 100)                        # no newline: must not grow forever
    assert f.overflows == 1
    assert f.feed(b"abc\n") == ["abc"]        # still functional afterwards
```

- [ ] **Step 2: 运行确认失败** — 三个新测试 FAIL
- [ ] **Step 3: 实现**

tcp.py：`_run_server` 的 handler 在接受连接时 `self._last_state = "connected"` 并发 connected 事件（原逻辑在 `_pump` 里已发，确保用转换判断包住）；finally 中改为：

```python
            finally:
                if self._last_state != "disconnected":
                    self._last_state = "disconnected"
                    self._on_event(self._name, "disconnected", "peer closed")
```

（`_pump` 内发 connected 处同步维护 `self._last_state = "connected"`——客户端模式修复时已有该字段，复用。）

can.py `run()` 主循环替换：

```python
    async def run(self) -> None:
        bus = self._bus
        reader = can.AsyncBufferedReader()
        notifier = can.Notifier(bus, [reader], loop=asyncio.get_running_loop())
        timeouts = 0
        try:
            while True:
                try:
                    msg = await asyncio.wait_for(reader.get_message(),
                                                 timeout=self._data_timeout)
                except asyncio.TimeoutError:
                    timeouts += 1
                    if timeouts == 1 and self._on_event:
                        self._on_event("can_link", "disconnected",
                                       f"no frames for {self._data_timeout:.0f}s")
                    if (self._bus_factory is not None
                            and timeouts >= self._reopen_after):
                        notifier.stop()
                        bus.shutdown()
                        bus = self._bus_factory()
                        reader = can.AsyncBufferedReader()
                        notifier = can.Notifier(bus, [reader],
                                                loop=asyncio.get_running_loop())
                        if self._on_event:
                            self._on_event("can_link", "reopened", "bus reopened")
                        timeouts = 0
                    continue
                if timeouts > 0 or not self._seen_any:
                    if self._on_event:
                        self._on_event("can_link", "connected", "")
                    self._seen_any = True
                    timeouts = 0
                self._on_frame(msg.arbitration_id, bytes(msg.data),
                               msg.timestamp or time.time())
        finally:
            notifier.stop()
```

（构造器存 `self._bus_factory`、`self._reopen_after`、`self._seen_any = False`；与现有 watchdog 字段合并，保留既有事件语义。）

gpchc.py：

```python
class LineFramer:
    def __init__(self, max_buf: int = 65536) -> None:
        self._buf = b""
        self._max = max_buf
        self.overflows = 0

    def feed(self, data: bytes) -> list[str]:
        self._buf += data
        if len(self._buf) > self._max and b"\n" not in self._buf:
            self._buf = b""
            self.overflows += 1
            return []
        *lines, self._buf = self._buf.split(b"\n")
        return [ln.strip().decode("ascii", "replace") for ln in lines if ln.strip()]
```

- [ ] **Step 4: 运行确认通过** — 新增 3 PASS，原有全部不破 + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "fix: transition-only listen events, CAN bus reopen, LineFramer buffer cap"`

---

### Task 12: App 接线（路 3/4 历元入库 + solver + 诊断循环 + 发布）

**Files:**
- Modify: `src/rtk_monitor/main.py`
- Test: `tests/test_app_plan2.py`

**Interfaces:**
- Consumes: Task 1–11 全部公开接口（签名见各任务 Interfaces）。
- Produces: `App` 新增属性 `epochs: EpochStore`、`event_machine: EventMachine`、`base_monitor: BaseStationMonitor`、`publisher: UdpPublisher | None`；内部 1Hz `_diagnosis_loop`（走 `_supervise`）。rtkrcv 仅当 `cfg.rtkrcv.binary` 非空时启用（manager + sol 采集）。**历元降采样**：gpchc/can 每源每秒最多写一条（记录上次写入秒）。

接线明细（实现即按此写）：
- `_on_sol`：裸流落盘不变；追加 `LineFramer` → `parse_gpchc` → 1Hz 决策 → `epochs.add(Epoch(t=host_time, src="gpchc", q=e.sat_status, sats=e.nsv1, age=e.diff_age, lat=..., lon=..., alt=..., heading=..., speed=...))`；
- `_on_can`：candump 落盘不变；追加 `Cgi610Assembler.feed` → 每完整周期 1Hz 决策 → `epochs.add(src="can", q=cyc.sat_status, sats=cyc.sats_used, age=cyc.diff_age, lat/lon/alt, sde/sdn/sdu=pos_sigma, heading, speed=vel[3])`；
- `_on_corr`：framer 输出里 `msg_type in (1005, 1006)` 时 `parse_1005(payload)` → `base_monitor.feed` → 存 `self._base_offset`；同时维护 `self._corr_last_t = time.time()`；
- rtkrcv 启用时：`RtkrcvManager(binary, run_dir=data_root/"rtkrcv", corr_port=reserve_corrections_port, obs_port=reserve_raw_obs_port, sol_port, on_event=self._on_event)` 走 `_supervise`；另起 `TcpCollector("rtkrcv_sol", "127.0.0.1", sol_port, on_data=self._on_rtksol, on_event=self._on_event)` 走 `_supervise`；`_on_rtksol` 用独立 `LineFramer` + `parse_llh_solution` → 存 `self._sol/self._sol_t` → `epochs.add(src="rtkrcv", ...)` ＋ `publisher.publish_fix(sol, heading=最近 gpchc/can 航向)`（1Hz 已由 rtkrcv 保证）；
- `_diagnosis_loop`（每 1s）：组装 `DiagnosisInput(now, corr_last_t, corr_age=self._sol.age if..., base_offset_m, sol, sol_t, sats=[], slip_count_30s=self._slips.count(now), divergence_m/since)` → `diagnose` → `event_machine.update(t, v, lat, lon)`；divergence：can 最新历元与 sol 皆存在且时间差 < 2s 时等距圆柱平面距离，超过 `3σ` 起计 `_div_since`，否则清零；`sats`/slip 本期由 stat 通道缺省为空（真机接 stat 流属 Task 14 清单，规则 4/6 自动不触发）；
- `event_machine` 的 `on_transition` → `publisher.publish_event`（publisher 仅 `cfg.publish.enabled` 时构造并 start）；
- `shutdown()` 追加：`epochs` 不关闭（与 events 同理由——测试/运行期查询）；publisher stop。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_app_plan2.py
import asyncio
import json
import socket
import struct
import textwrap

from rtk_monitor.config import load_config
from rtk_monitor.main import build_app
from rtk_monitor.parsers.rtcm import crc24q


def _cfg(tmp_path, udp_port, corr_port):
    p = tmp_path / "config.yaml"
    p.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: {corr_port}}}
        raw_obs: {{host: 127.0.0.1, port: 1, listen: false}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:p2app
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        publish: {{enabled: true, host: 127.0.0.1, port: {udp_port}}}
        diagnosis: {{corr_gap_s: 0.5, close_hysteresis_s: 0.5}}
        """))
    return load_config(p)


def _gpchc_line():
    body = ("GPCHC,2372,113755.36,174.20,1.25,-0.80,0.12,-0.05,0.30,"
            "0.0123,-0.0045,0.9987,44.50123456,90.28765432,617.123,"
            "0.02,-0.01,0.00,0.02,39,38,42,1.2,0")
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}\r\n".encode()


async def test_epochs_and_corr_outage_event(tmp_path):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0)); rx.setblocking(False)
    # corrections server: sends one 1074 frame then goes silent -> outage
    payload = (1074 << 4).to_bytes(2, "big") + b"\x00" * 6
    head = bytes([0xD3, 0x00, len(payload)])
    frame = head + payload + crc24q(head + payload).to_bytes(3, "big")

    async def corr_handler(reader, writer):
        writer.write(frame)
        await writer.drain()
        await asyncio.sleep(30)
    corr_srv = await asyncio.start_server(corr_handler, "127.0.0.1", 0)
    corr_port = corr_srv.sockets[0].getsockname()[1]

    app = build_app(_cfg(tmp_path, rx.getsockname()[1], corr_port))
    task = asyncio.create_task(app.run_forever())
    await asyncio.sleep(0.1)
    # feed route 3 (GPCHC) via listen port
    _, w = await asyncio.open_connection("127.0.0.1", app.sol_collector_port())
    w.write(_gpchc_line()); await w.drain()
    await asyncio.sleep(2.5)          # > corr_gap_s: outage rule must fire
    task.cancel()
    await app.shutdown()

    assert app.epochs.latest("gpchc").sats == 39
    codes = [e.code for e in app.events.query() if e.etype == "diagnosis"]
    assert "corr_outage" in codes
    # event also published on UDP
    loop = asyncio.get_running_loop()
    seen = []
    try:
        while True:
            data = await asyncio.wait_for(loop.sock_recv(rx, 4096), 0.3)
            seen.append(json.loads(data.decode()))
    except asyncio.TimeoutError:
        pass
    assert any(m["type"] == "gnss_event" and m["event"] == "corr_outage" for m in seen)
    w.close(); corr_srv.close(); rx.close()
```

（`sol_collector_port()`：App 暴露路 3 监听端口的小助手——监听模式下 `TcpCollector.bound_port`，App 需保存路 3 collector 引用并提供该方法，测试与 Plan 3 界面都要用。）

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现** — 按"接线明细"修改 main.py：新增 imports（EpochStore/Epoch、EventMachine、diagnose/DiagnosisInput、BaseStationMonitor、UdpPublisher、RtkrcvManager、parse_llh_solution、LineFramer、parse_gpchc、Cgi610Assembler、parse_1005、SlipWindow）；`App.__init__` 构造 epochs/base_monitor/event_machine/publisher（enabled 时）/assembler/framers/状态字段（`_corr_last_t=None`、`_sol=None`、`_sol_t=None`、`_base_offset=None`、`_div_since=None`、`_slips=SlipWindow()`、每源上次写秒 dict）；`run_forever` 追加 `_supervise("diagnosis", self._diagnosis_loop)`、publisher.start、rtkrcv 启用时 manager+sol collector 的 `_supervise`。等距圆柱距离助手：

```python
def _horiz_dist_m(lat1, lon1, lat2, lon2):
    import math
    r = 6378137.0
    x = math.radians(lon2 - lon1) * r * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1) * r
    return math.hypot(x, y)
```

`_diagnosis_loop` 骨架：

```python
    async def _diagnosis_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            now = time.time()
            div_m = None
            can_e = self.epochs.latest("can")
            if (self._sol is not None and can_e is not None and can_e.lat is not None
                    and abs((self._sol_t or 0) - can_e.t) < 2.0):
                div_m = _horiz_dist_m(self._sol.lat, self._sol.lon, can_e.lat, can_e.lon)
                sigma = max(1e-3, (self._sol.sdn ** 2 + self._sol.sde ** 2) ** 0.5)
                if div_m > self.cfg.diagnosis.divergence_sigma * sigma:
                    self._div_since = self._div_since or now
                else:
                    self._div_since = None
            inp = DiagnosisInput(
                now=now, corr_last_t=self._corr_last_t,
                corr_age=self._sol.age if self._sol else
                         (can_e.age if can_e else None),
                base_offset_m=self._base_offset,
                sol=self._sol, sol_t=self._sol_t,
                sats=[], slip_count_30s=self._slips.count(now),
                divergence_m=div_m, divergence_since=self._div_since)
            v = diagnose(inp, self.cfg.diagnosis)
            lat = self._sol.lat if self._sol else (can_e.lat if can_e else None)
            lon = self._sol.lon if self._sol else (can_e.lon if can_e else None)
            self.event_machine.update(now, v, lat, lon)
```

全部新逻辑放在既有 try/except 防护风格内（回调守卫沿用 Plan 1 模式）。

- [ ] **Step 4: 运行确认通过** — `pytest tests/test_app_plan2.py -v` PASS + 全套（含 Plan 1 e2e 不破）
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: wire epochs, solver, diagnosis loop and publisher into App"`

---

### Task 13: fake rtkrcv 与解算链路端到端

**Files:**
- Create: `tests/fake_rtkrcv.py`（可执行）
- Test: `tests/test_e2e_solver.py`

**Interfaces:**
- Consumes: `RtkrcvManager` 的 env 约定（`RTKRCV_SOL_PORT`）；App 全装配。
- Produces: 无新生产接口；证明 binary→solution→epochs→publish 链路。

- [ ] **Step 1: 写 fake 与失败测试**

```python
#!/usr/bin/env python3
# tests/fake_rtkrcv.py — stand-in for rtkrcv: serve llh lines on RTKRCV_SOL_PORT.
import os
import socket
import threading
import time

LINE = ("2026/08/27 04:15:55.400   44.501234567   90.287654321   617.1234"
        "   1  38   0.0110   0.0123   0.0322  -0.0001   0.0002   0.0003"
        "   0.80   25.0\r\n").encode()


def serve(conn):
    try:
        while True:
            conn.sendall(LINE)
            time.sleep(0.2)
    except OSError:
        pass


def main():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", int(os.environ["RTKRCV_SOL_PORT"])))
    srv.listen()
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
```

```python
# tests/test_e2e_solver.py
import asyncio
import sys
import textwrap
from pathlib import Path

from rtk_monitor.config import load_config
from rtk_monitor.main import build_app

FAKE = Path(__file__).parent / "fake_rtkrcv.py"


async def test_solver_chain_writes_rtkrcv_epochs(tmp_path):
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 1}}
        raw_obs: {{host: 127.0.0.1, port: 1}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:solvertest
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        rtkrcv: {{binary: "{sys.executable}", sol_port: 0, extra_args: ["{FAKE}"]}}
        """))
    app = build_app(load_config(cfg_file))
    task = asyncio.create_task(app.run_forever())
    for _ in range(100):
        await asyncio.sleep(0.1)
        if app.epochs.latest("rtkrcv") is not None:
            break
    task.cancel()
    await app.shutdown()
    e = app.epochs.latest("rtkrcv")
    assert e is not None and e.q == 1 and e.sats == 38
    assert abs(e.lat - 44.501234567) < 1e-9 and abs(e.ratio - 25.0) < 1e-6
```

注意两点接线要求（属 Task 12 代码，此处验证）：(a) `rtkrcv.sol_port: 0` 时 App 需在启动前自选一个空闲端口（`socket` bind 0 再取端口）传给 manager 与 sol collector——实现放在 `build_app`/App 初始化里；(b) `extra_args` 里的脚本路径作为 `binary` 的参数传递（manager 已支持 `*extra_args`），使 `python fake_rtkrcv.py` 可被 spawn。fake 忽略 `-s -nc -o conf` 参数？——**不忽略会出错**：fake 的 `main()` 不解析 argv，直接忽略所有位置参数即可（Python 脚本不读 argv 就无影响，但 manager 命令行是 `python -s -nc -o conf fake.py`，`-s`/`-nc` 会被 python 解释器吞掉且合法（-s 合法、-nc 非法）。**因此 manager 需调整**：`extra_args` 放在 binary 之后、固定参数之前——即命令行为 `binary *extra_args -s -nc -o conf`。Task 10 实现时就按此顺序写（`self._binary, *self._extra, "-s", "-nc", "-o", str(conf)`），Task 10 的测试不受影响，真实 rtkrcv 也不受影响（extra_args 默认空）。

- [ ] **Step 2: 运行确认失败** — FAIL
- [ ] **Step 3: 实现** — `chmod +x tests/fake_rtkrcv.py`；按上述两点补齐 Task 12 的 App 端口自选逻辑与 Task 10 的参数顺序（若 Task 10 已按此实现则无改动）。
- [ ] **Step 4: 运行确认通过** — PASS + 全套
- [ ] **Step 5: Commit** — `git add -A && git commit -m "test: fake rtkrcv end-to-end solver chain"`

---

### Task 14: 构建脚本与真机集成清单

**Files:**
- Create: `scripts/build_rtkrcv.sh`
- Create: `docs/integration-rtkrcv.md`

**Interfaces:** 无代码接口；无测试依赖（文档任务，一个 commit）。

- [ ] **Step 1: 写构建脚本**

```bash
#!/usr/bin/env bash
# Build rtklibexplorer demo5 rtkrcv into tools/bin/rtkrcv (host or ARM64).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/RTKLIB"
if [ ! -d "$SRC" ]; then
    git clone --depth 1 --branch demo5 https://github.com/rtklibexplorer/RTKLIB.git "$SRC"
fi
make -C "$SRC/app/consapp/rtkrcv/gcc" -j"$(nproc)"
mkdir -p "$ROOT/tools/bin"
cp "$SRC/app/consapp/rtkrcv/gcc/rtkrcv" "$ROOT/tools/bin/rtkrcv"
echo "built: $ROOT/tools/bin/rtkrcv"
```

- [ ] **Step 2: 写集成清单 docs/integration-rtkrcv.md**（要点，写成 checklist 文档）：

```markdown
# rtkrcv 真机集成核对清单

上车/联调时逐项核对（单元测试覆盖不到的部分）：

- [ ] `scripts/build_rtkrcv.sh` 在目标机（ARM64）构建通过；`tools/bin/rtkrcv -h` 可运行
- [ ] config.yaml `rtkrcv.binary` 指向该二进制；启动后 `ps` 可见进程、事件表出现 rtkrcv connected
- [ ] 核对 conf 键名与 demo5 版本一致（`rtkrcv.conf` 由程序生成在 `<data_root>/rtkrcv/`）：
      inpstr1/2、outstr1、pos1-posmode、pos2-armode——不一致则改 `solver/rtkrcv.py` 模板
- [ ] `-nc` 参数在该版本可用（禁用控制台）；不可用则从命令行移除并验证后台运行
- [ ] 板卡原始观测格式确认：`convbin -scan` 判别 RTCM3 还是华测二进制；
      非 RTCM3 时改 conf `inpstr1-format`
- [ ] solution 流通：`nc 127.0.0.1 <sol_port>` 能看到 llh 行；epochs 表出现 src='rtkrcv'
- [ ] solution status（规则 4/6 的输入）：确认 demo5 rtkrcv 输出 $SAT 的配置方式
      （outstat 相关键），接通后把 stat 流喂给 `parse_sat_line`/`SlipWindow`——
      当前 App 的 `sats`/slip 输入为空，规则 4/6 不触发，属预期降级
- [ ] 固定率合理性：开阔地静止 5 分钟，Q=1 占比 > 90%
```

- [ ] **Step 3: chmod +x scripts/build_rtkrcv.sh；.gitignore 追加 `third_party/` 与 `tools/bin/`**
- [ ] **Step 4: 全套测试确认不破**
- [ ] **Step 5: Commit** — `git add -A && git commit -m "docs: rtkrcv build script and field integration checklist"`

---

## Self-Review 记录

- **Spec 覆盖**：§4.1 rtkrcv 配置（T10/T14）、§4.2 七条规则与默认阈值（T1/T6）、§4.3 事件状态机含迟滞（T7）、基线学习 10 分钟中位数+持久化+人工 reset（T8）、§3.1 路 3/4 解析入 SQLite（T4/T12）、§7 UDP 发布（T9/T12）、§2.1 solver/diagnosis/publisher/store 模块边界（目录结构）、Plan 1 停放项（T11）。**已知降级**：规则 4/6 的逐卫星输入依赖 rtkrcv stat 流，其真机配置方式待 T14 清单确认，本期 App 传空列表（规则不触发）——spec §4.2 功能保留、数据源后补，已在 T12/T14 明示。天空图逐卫星摘要（§6 `sats_json`）表列已建，写入依赖 stat 流，同上后补。
- **占位符扫描**：无 TBD/TODO；所有代码步骤含完整代码。
- **类型一致性**：`RtkSolution` 字段（T2 定义，T6/T9/T12/T13 消费）一致；`Verdict(level, code, message)`（T6→T7/T9/T12）一致；`Epoch`/`EpochStore.add/latest/query`（T4→T12/T13）一致；`EventStore.record(..., level, code, lat, lon)`/`close_event`（T5→T7）一致；`RtkrcvManager` 参数顺序含 T13 修正（extra_args 在固定参数前）已回写 T10 说明；`CanCollector` 新参默认值向后兼容（T11）；`DiagnosisCfg` 字段名（T1）与 `diagnose` 用法（T6）一致。
