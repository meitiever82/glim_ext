# Golden Parity 用例格式规范（v1）

期 0 产物。golden parity 的目标：**同一份参数 + 同一份输入 → 新旧链路输出一致**。
本规范定义用例文件格式、录制/回放语义、断言容差。

## 1. 文件组织

```
golden/
├── README.md                # 本规范
├── cases/                   # 用例（人工编写，评审对象）
│   ├── by_001_*.yaml
│   └── hc_001_*.yaml
└── golden/                  # golden 输出（工具录制，禁止手改）
    └── by_001_*.golden.yaml
```

- `cases/` 只有输入与意图，**人工编写**，进评审。
- `golden/` 由录制器从**线上旧代码**跑出，带溯源头（git commit / 配置哈希 / 录制时间），
  **禁止人工修正**——发现不符预期的 golden 意味着发现了线上行为或 bug，走评审改类别。

## 2. 用例 schema

```yaml
case: by_001_rtkfixed_basic     # 全局唯一，by_=北云 hc_=华测
class: keep                     # keep | changed | dropped
brand: beiyun                   # beiyun | huace
description: RTK 固定解直通
# class=changed 时必填：指向方案文档中的废弃项/变更项编号
change_ref: ""                  # 例: "deprecate-roll-pitch-override"

config:                         # 配置快照——录制时线上 localization_parser.yaml 的全量值
  origin_lat: 44.50323451       # 等价性在固定参数下断言；改参数不改用例
  ...

frames:                         # 带时间轴的帧序列；t 为相对秒（仿真时钟）
  - t: 0.0
    type: gpgga                 # 结构化类型，录制器合成协议字节（含校验和）
    fields: {sats: 18, age: 1.0, lat: 44.5032, lon: 92.9175, alt: 753.8}
  - t: 0.02
    type: inspvaxa
    fields: {ins_status: INS_SOLUTION_GOOD, pos_type: INS_RTKFIXED, ...}

# keep: 不写 expect（期望=golden 录制值）
# changed: expect_new 手写新语义断言；golden 里仍保留旧输出存档
expect_new:
  - at_frame: 1
    is_valid: true
    fault_code: None
```

## 3. 三类用例的断言语义

| class | 断言内容 | 用途 |
|---|---|---|
| `keep` | 新 Judge 输出 == golden（旧代码录制值） | 保留行为，占大多数 |
| `changed` | golden 存档旧输出不动；新 Judge 输出 == 手写 `expect_new`（新语义） | 已知要改的行为 |
| `dropped` | golden 存档；新链路断言**不再产生该行为** | 废弃行为（如 fusion 覆写） |

`changed`/`dropped` 必须填 `change_ref`，期 4 清理时连同用例一起删。

## 4. 回放与容差

- **时钟**：新 Judge 接收注入时间（用例 t 轴），回放零墙钟耗时、完全确定。
  录制器跑旧代码用真实时钟（旧实现写死 steady_clock/system_clock，无法注入），
  用例 t 轴即真实相对时间——30s 粘滞用例录制时真的等 30s。
- **帧配对**：按帧序号（输入序号）配对，不是到达时间。
- **精确相等字段**：`is_valid` / `rtk_status` / `fault_code` / `ins_status` / `pos_type`。
- **数值容差**：位置/速度/姿态 1e-9（deg 或 m），只为吸收浮点运算顺序差异；
  卫星数/龄期等整数语义字段精确相等。
- **豁免字段**（白名单）：`timestamp`（发布时刻）、`gps_time`（显示字符串）、
  IMU 缩放中间值。白名单之外的任何差异 = 分歧。
- **容差击穿处置**：任何字段超容差即判定分歧，**禁止当场放宽容差**；
  需放宽容差时提交评审，记录理由与新容差，版本化变更。

## 5. golden 版本化

每个 golden 文件头：

```yaml
provenance:
  recorder_version: 1
  source_repo: localization
  source_commit: <git commit>
  config_sha256: <config 快照的哈希>
  recorded_at: <ISO8601>
```

回放时校验：源仓库当前 commit 与 golden 记录不同 → 警告"基线可能过期"，
由人确认继续或重录。**基线过期不自动重录**。

## 6. 范围说明

- 处理器级覆盖 fault_code 1–8 的全部触发分支（golden 能录到的层级）。
- fault 9（NoData，节点级"从未收到首帧"）不在处理器层，由新 Judge 单测覆盖，
  不进 golden。
- 节点级 50Hz 定频重发不参与 parity：golden 比较的是**状态序列**（每次输入
  产生的最新状态），不是发布实例数。
