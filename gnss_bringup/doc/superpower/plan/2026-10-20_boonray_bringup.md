# boonray_bringup（定位链路重构 · 第 0+1 期）Implementation Plan

> **状态：已完成（2026-11-16）**。本 plan 的 Task 3–6 在 plan 编写前已由另一轮实现落地为 `gnss_core/include/gnss_core/judge.hpp/cpp` 与 `localization/golden/` YAML golden 体系；本轮完成 Task 1、2、7 并补齐文档。实际 API 与原始 plan 有合理差异，详见下方「实现差异说明」。
> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 冻结 RtkFix/LocalizationData 消息扩展，并把定位门控规则从 localization_parser 抽离为 gnss_core 纯库 `Judge(Evidence, GateState&) → Verdict`，用表征测试（golden）证明与现行行为逐点一致。

**Architecture:** 规则唯一存放在 gnss_core 纯库（无 ROS），本计划只产库和消息，不动任何线上节点；golden 表征测试从现行 processor 捕获行为基线，新库对其逐帧对齐。

**Tech Stack:** C++17 / ROS2 Humble / rosidl（msg）/ GTest / ament+纯CMake 双路径

**Spec:** `gnss_bringup/doc/superpower/spec/2026-10-20_boonray_bringup.md`（架构图 `../img/gnss_target_arch.png`）

## Global Constraints

- 本计划**不修改 localization_parser 线上行为**——golden 捕获工具只新增测试代码，不动 src/
- 行为逐点保留：分支顺序、多因并存取最大枚举优先级、单点解龄期宽限、rtk_status 与门控解耦（门控永不压 rtk_status）、**粘滞宽限**（无分支命中时沿用上一帧，但限 sticky_grace_time=30s，超时判无效 + GraceTimeout + rtk_status 按实际解类型修正——2026-10-20 合入的 152519 新行为，beiyun_processor.cpp:239-271）
- FaultCode 枚举值必须与 `robominer_msgs/msg/LocalizationDataAux.msg` 注释对齐：0 NONE / 1 INS_NOT_CONVERGED / 2 GRACE_TIMEOUT / 3 HARDWARE_WARNING / 4 SATELLITE_INSUFFICIENT / 5 BASELINE_ABNORMAL / 6 ENU_OUT_OF_BOUNDS / 7 PARSE_FAIL / 8 DATA_TIMEOUT / 9 NO_DATA
- invalid_reason 枚举对齐飞书文档：0 有效 / 1 设备异常 / 2 初始化未完 / 3 差分龄期大 / 4 卫星信号差 / 5 解算结果异常 / 99 其他
- gnss_msgs 改动后三仓必须同步重建：finder_ros(driver_ws)、glim_ws、boonray_ws（DDS 静默不匹配风险）
- 阈值默认值从 `localization_parser/config/localization_parser.yaml` 原样拷贝：`xyz_threshold: 10000`、`stars_threshold: 10`、`heading_length: 1.6`、`heading_length_threshold: 0.2`、`single_point_age_threshold: 10.0`、`rtk_float_grace_time: 20.0`、`unfixed_time: 5.0`
- gnss_core 测试沿用现有约定：ament 路径用 ament_add_gtest，纯 CMake 路径用 GTest::gtest_main + add_test（CMakeLists.txt:7-8 注释），测试文件放 `test/`
- colcon 构建统一加 `--cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3`（本机 Hermes Python 会抢占）

## Review Focus

1. **首帧无历史**：GateState 无任何上一帧时，粘滞逻辑退化为 `is_valid=false / rtk_status=RTKINIT / fault_code=None`（beiyun_processor.cpp:201-204）——golden 必须含首帧用例
2. **GGA age 永不过期**：`data.age` 仅由 GPGGA 填充且不随时间失效，未收到 GGA 时保持 0（beiyun_processor.cpp:139-143）——"age 为旧值"不等于"无 age"
3. **ins_status 未匹配粘滞**：字符串未命中已知值时 aux.ins_status 保持上一帧，首帧未匹配时为 InsInactive（beiyun_processor.cpp:166-175）
4. **星数门控无豁免**：GPFPD nsv1 与 HEADINGA obs 任一低于阈值即判无效，未收到 GPFPD 时 nsv1=0 必触发（beiyun_processor.cpp:249-255）——golden 需含"无 GPFPD"用例
5. **华测宽限按模式分档**：类型5（浮点）用 rtk_float_grace_time=20s，组合导航模式用 combined_nav_mod_time，其余 unfixed_time=5s（huace_processor.cpp:262-277）——golden 需覆盖三档
6. **北云粘滞宽限计时用 steady_clock**（单调钟，不受系统时间跳变影响，beiyun_processor.cpp:248-257）；华测宽限用系统墙钟（localization_node.cpp:199-200 注释）——Judge 的时间输入必须显式由调用方注入，库内不读时钟；两个品牌的钟语义不同，Evidence 须分别携带或由适配器统一

---

### Task 1: gnss_msgs RtkFix 扩展 + 三仓重建

**Files:**
- Modify: `finder_ros/drivers/gnss_msgs/msg/RtkFix.msg`
- Modify: `finder_ros/drivers/gnss_msgs/CMakeLists.txt`（如 msg 列表为显式枚举）

**Interfaces:**
- Produces: 扩展后的 `gnss_msgs/msg/RtkFix`，新增字段（全部沿用"0/空 = 源不提供"惯例）：
  - `float32 vel_east` / `float32 vel_north` / `float32 vel_up`（m/s，ENU）
  - `float32 roll` / `float32 pitch`（deg；heading 已有）
  - `float32 gyro_x/y/z`（deg/s）、`float32 acc_x/y/z`（m/s²）——IMU（北云 CORRIMUDATA 缩放后）
  - `uint8 ins_status`（枚举值镜像 LocalizationDataAux.ins_status 注释 0-10，另加常量 INS_UNKNOWN=255）
  - `float32 baseline_length`（m）、`uint8 heading_type`（0 NONE / 1 NARROW_INT / 2 其他）
  - `float64 undulation`（m）

- [x] **Step 1: 编辑 RtkFix.msg 追加上述字段**，每个字段带一行注释（单位/来源/缺省约定）
- [x] **Step 2: 三仓重建验证**

```bash
# boonray_ws
cd /home/boonraygrn/boonray_ws && source /opt/ros/humble/setup.bash && \
  colcon build --packages-select gnss_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
# glim_ws 与 driver_ws 同样重建 gnss_msgs
```

Expected: 三仓均编译通过
- [x] **Step 3: pub/sub 冒烟测试**：`ros2 interface show gnss_msgs/msg/RtkFix` 验证新字段可见（DDS 不匹配时此步静默失败，正是要防的）。由于本机 DSH sandbox 阻止 ROS2 写 `~/.ros/log`，未跑 `ros2 topic pub/echo`，但接口定义已确认包含全部新增字段。
- [ ] **Step 4: Commit**（finder_ros 仓，`git -C ~/Documents/GitHub/ztpilot/finder_ros add drivers/gnss_msgs && git commit -m "feat: RtkFix 扩展速度/姿态/IMU/ins_status/基线字段"`）

### Task 2: robominer_msgs LocalizationData 扩展 + LocalizationMetrics 新消息

**Files:**
- Modify: `boonray_ws/src/common/robominer_msgs/msg/LocalizationData.msg`
- Create: `boonray_ws/src/common/robominer_msgs/msg/LocalizationMetrics.msg`

**Interfaces:**
- Produces: `LocalizationData` 追加：
  - `uint8 invalid_reason`（常量 LOC_VALID=0 / DEVICE_FAULT=1 / INIT_NOT_READY=2 / DIFF_AGE_TOO_LARGE=3 / SATELLITE_WEAK=4 / SOLUTION_ABNORMAL=5 / OTHER=99）
  - `bool zero_velocity`
  - `float64 static_std_horizontal` / `float64 static_std_vertical`（m，-1 = 未产出）
- Produces: `LocalizationMetrics.msg`：`std_msgs/Header header`、`float64 rtk_fix_ratio`（-1=未达有效时长）、`float64 fix_time_s`、`float64 total_time_s`、`float64 static_std_horizontal`、`float64 static_std_vertical`、`float64 static_max_deviation`、`uint32 segment_count`

- [x] **Step 1: 写失败测试**：`ros2 interface show robominer_msgs/msg/LocalizationData` 应含 invalid_reason（构建前失败）
- [x] **Step 2: 编辑两个 msg 文件**（LocalizationData.msg 追加尾部，不动现有字段顺序；新建 LocalizationMetrics.msg），CMakeLists.txt/msg 列表登记新消息
- [x] **Step 3: 重建验证**

```bash
cd /home/boonraygrn/boonray_ws && source /opt/ros/humble/setup.bash && \
  colcon build --base-paths src/common --packages-select robominer_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
ros2 interface show robominer_msgs/msg/LocalizationData | grep -c 'invalid_reason\|zero_velocity\|static_std'   # 期望 ≥4（3字段+常量行）
```

- [x] **Step 4: 全量重建 boonray_ws 确认下游包（flofix_loc 等）不受影响**
  - 已验证 `localization_parser`、`flofix_loc` 编译通过。
  - 完整 workspace build 被 `src/fusion_localization/record_bag/vendor/robominer_msgs`（ROS1 catkin 包，与 common/robominer_msgs 同名）阻塞，需用 `--base-paths src/common ...` 或 `--packages-ignore` 规避。

Expected: 全部包编译通过（msg 尾部追加，二进制兼容）
- [ ] **Step 5: Commit**（common 仓，中文提交信息写清字段语义）

### Task 3: gnss_core 判决类型定义（gate.hpp）

**Files:**
- 实际实现为 `gnss_core/include/gnss_core/judge.hpp`（非 plan 原定的 `gate.hpp`）

**Interfaces:**
- 已落地的 API 与 plan 差异：
  - namespace 为 `gnss_core::judge`（而非 `gnss_core`）
  - 结构体名为 `Evidence` / `Config` / `Verdict` / `GateState`（plan 原定 `GateConfig` / `InvalidReason` 等）
  - 函数为 `judge_beiyun()` / `judge_huace()`（plan 原定 `Judge()` / `JudgeHuace()`）
  - `Verdict` 当前不含 `invalid_reason`；`invalid_reason` 映射在 `judge_replay` 工具中按需本地实现
- 核心语义不变：纯函数、无 ROS、时间注入、GateState 显式化。

- [x] **Step 1–5: 已完成并编译通过**（见 `judge.hpp`、`src/judge.cpp`、`test/test_judge.cpp`）

### Task 4: golden 表征捕获（北云 + 华测）

**Files:**
- 实际实现为 `localization/golden/` 目录：
  - `cases/by_*.yaml`、`cases/hc_*.yaml`：人工编写输入用例
  - `golden/*.golden.yaml`：旧 processor 录制输出
  - `tools/golden_recorder.cpp`：从旧代码录制 golden
  - `tools/golden_replay.cpp`：驱动 `gnss_core::judge` 逐帧对比

**Interfaces:**
- 与 plan 原定 CSV 格式不同，但实际覆盖北云 10 条 + 华测 8 条 = 18 条用例，全部 keep 类断言通过。

- [x] **Step 1–4: 已完成并验证通过**

### Task 5: Judge 北云规则实现（对齐 golden）

**Files:**
- `gnss_core/src/judge.cpp`
- `gnss_core/test/test_judge.cpp`

- [x] **Step 1–5: 已完成**
  - `test_judge` 17 个用例全部 PASS
  - golden parity 18 个用例全部零分歧

### Task 6: Judge 华测规则实现（对齐 golden）

**Files:**
- `gnss_core/src/judge.cpp`
- `gnss_core/test/test_judge.cpp`

- [x] **Step 1–4: 已完成**
  - 华测分支 6 个单测全部 PASS
  - golden parity 华测 8 个用例全部零分歧

### Task 7: 回放 CLI 骨架（judge_replay）

**Files:**
- Create: `gnss_core/tools/judge_replay.cpp`
- Modify: `gnss_core/CMakeLists.txt`（tools 段，参照 `add_executable(synth_rtk_fix ...)` 现有写法）

**Interfaces:**
- Consumes: `judge_beiyun` / `judge_huace`；输入为品牌原始 ASCII 流
- Produces: CLI `judge_replay <brand> <raw.log> [--origin lat,lon,alt] > verdicts.csv`
- 输出列：`gps_sec,is_valid,rtk_status,fault_code,invalid_reason`

- [x] **Step 1: 写测试**：`gnss_core/tests/test_judge_replay.py` 构造北云/华测样本流，断言输出格式
- [x] **Step 2: 实现骨架并通过测试**
- [ ] **Step 3: 用车上现有落盘文件实跑一次**（车上路径未挂载到本机，留待现场执行）
- [ ] **Step 4: Commit** `feat(gnss_core): judge_replay 回放 CLI 骨架`

---

## 实现差异说明

1. **judge API 命名与结构**：原始 plan 预定 `gate.hpp` + `Judge()`/`JudgeHuace()` + `GateConfig`/`InvalidReason`；实际已落地为 `judge.hpp` + `judge_beiyun()`/`judge_huace()` + `Config`/`Verdict`。语义等价，仅命名/组织差异。
2. **golden 格式**：原始 plan 预定 CSV golden；实际已落地为 YAML 用例 + YAML golden + `golden_recorder/golden_replay` 工具，更易人工评审与版本化。
3. **invalid_reason**：原始 plan 预定 `Verdict` 内含 `invalid_reason` 并由 `MapInvalidReason()` 统一映射；当前 `Verdict` 仅含 `is_valid/rtk_status/fault_code`。`judge_replay` 工具按 plan 映射表在输出层补充 `invalid_reason`，不影响 judge 核心。
4. **三仓同步**：glim_ws、driver_ws 的 `gnss_msgs` 已重建；boonray_ws 的 `robominer_msgs` 已重建。下游 `gnss_bringup`（glim_ws）、`localization_parser`、`flofix_loc`（boonray_ws）均编译通过。

## 验证结果（2026-11-16）

- `colcon build --packages-select gnss_core` ✅
- `colcon build --packages-select gnss_bringup` ✅
- `colcon build --packages-select gnss_msgs` in glim_ws / driver_ws ✅
- `colcon build --base-paths src/common --packages-select robominer_msgs` ✅
- `colcon build --base-paths src/common src/localization --packages-select localization_parser` ✅
- `colcon build --base-paths src/common src/flofix_loc --packages-select flofix_loc` ✅
- `gnss_core/test_judge` 17 tests PASSED ✅
- `gnss_core/tests/test_judge_replay.py` 12 tests PASSED ✅
- `localization/golden` 18 cases parity zero mismatch ✅
- `ros2 interface show gnss_msgs/msg/RtkFix` 含全部新增字段 ✅
- `ros2 interface show robominer_msgs/msg/LocalizationData` 含 invalid_reason/zero_velocity/static_std_* ✅
- `ros2 interface show robominer_msgs/msg/LocalizationMetrics` 存在且字段完整 ✅

---

# Phase 2 补充记录（2026-11-17，上车测试前）

## 实现内容

Phase 2 的核心目标（适配器 + judge + 对比/metrics 节点）已通过现有 `localization_bridge` 节点与本轮新增的 `metrics_node`、`divergence_node` 覆盖：

| 设计目标 | 实现方式 | 文件 |
|---|---|---|
| 北云适配 + judge | `localization_bridge` 直接订阅 OEM7 驱动话题，内部完成字段映射与 `LocalizationGate` 判决，发布 `LocalizationData`/`_aux` | `src/localization_bridge_node.cpp`, `include/gnss_bringup/localization_gate.hpp` |
| 慢变 metrics | `metrics_node` 订阅 `LocalizationData`，计算固定率、零速段静态 σ，发布 `LocalizationMetrics` | `src/metrics_node.cpp` |
| 双路对比 divergence | `divergence_node` 订阅两路 `RtkFix`，用 `gnss_core::DivergenceMonitor` 学习经验 σ 并发布 `/diagnostics` | `src/divergence_node.cpp` |
| 610 / 华测适配 | **未实现**。当前链路只覆盖北云 OEM7。若明天车端使用 610，需要临时启用旧 `localization_parser` 或紧急补一个 `c610_adapter` |

## 启动方式（明天上车）

```bash
source /opt/ros/humble/setup.bash
source <boonray_ws>/install/setup.bash   # 提供 robominer_msgs
source <glim_ws>/install/setup.bash

# 只起定位链（北云已有驱动话题时）
ros2 launch gnss_bringup gnss_bringup.launch.py \
  enable_bridge:=false \
  enable_localization:=true \
  enable_metrics:=true \
  enable_divergence:=false \
  localization_params_file:=<现场 yaml 路径>

# 如需对比 rtkrcv 独立解与板卡融合解，确认两路 RtkFix 话题存在后：
# enable_divergence:=true
```

## 验证结果

- `colcon build --packages-select gnss_bringup` ✅（需 boonray_ws overlay 提供 robominer_msgs）
- `test_localization_gate` 19 tests PASSED ✅
- `test_localization_mapping` 18 tests PASSED ✅
- `test_loc_alarm_state` 11 tests PASSED ✅
- `localization/golden` 18 cases parity zero mismatch ✅
- launch 文件 Python 语法检查通过 ✅
- `install/gnss_bringup/lib/gnss_bringup/{localization_bridge,metrics_node,divergence_node}` 均存在 ✅

## 上车前必须确认

1. **北云驱动话题名**与 `config/gnss_bringup_localization.yaml` 中 `topics.inspvax/bestpos/corrimudata/heading2` 一致。
2. **ENU 原点** `origin_lat/lon/height` 与北云驱动 map 原点一致，否则 `world→map` 静态 TF 失效、地图整体平移。
3. **IMU 频率** `imu_rate_hz` 与整车 CORRIMU 输出频率一致（当前默认 100 Hz）。
4. **rtkrcv 独立解话题** `/gnss/rtkfix_independent` 需要在开 `enable_divergence:=true` 时存在；否则 divergence 节点只会报 "none" 状态，不会崩溃。
5. 若车辆使用 **610（华测）**，Phase 2 未完成，需回退到旧 `localization_parser` 链路。

## 仍属 Plan 3/4 未做项

- 下游订阅统一切换到 `/localization/localization_data`（取决于 flofix_loc/控制是否已改好）
- `soft_dtu` 删除、旧录制废止
- `localization_parser` 仓转只读

## Self-Review 记录

- **Spec 覆盖**：本 plan 覆盖 SPEC 第 0+1 期（msg 冻结 + Judge 抽库 + parity 工具）。metrics 库、适配器节点、divergence 接入属 Plan 2（第 2 期），不在此列
- **VCU**：按决定本期不接入，Evidence 无 VCU 字段
- **类型一致性**：Task 3 冻结的枚举值与 LocalizationDataAux.msg 注释、飞书 invalid_reason 枚举逐项核对过
- **比例**：实现主体是规则迁移（golden 驱动），plan 刻意不含函数体
