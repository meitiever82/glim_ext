# boonray_bringup（定位链路重构 · 第 0+1 期）Implementation Plan

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

- [ ] **Step 1: 编辑 RtkFix.msg 追加上述字段**，每个字段带一行注释（单位/来源/缺省约定）
- [ ] **Step 2: 三仓重建验证**

```bash
# boonray_ws
cd /home/boonraygrn/boonray_ws && source /opt/ros/humble/setup.bash && \
  colcon build --packages-select gnss_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
# glim_ws 与 driver_ws 同样重建 gnss_msgs
```

Expected: 三仓均编译通过
- [ ] **Step 3: pub/sub 冒烟测试**：boonray_ws 下 `ros2 topic pub` 一个扩展字段非零的 RtkFix，`ros2 topic echo --once` 验证新字段可见（DDS 不匹配时此步静默失败，正是要防的）
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

- [ ] **Step 1: 写失败测试**：`ros2 interface show robominer_msgs/msg/LocalizationData` 应含 invalid_reason（构建前失败）
- [ ] **Step 2: 编辑两个 msg 文件**（LocalizationData.msg 追加尾部，不动现有字段顺序；新建 LocalizationMetrics.msg），CMakeLists.txt/msg 列表登记新消息
- [ ] **Step 3: 重建验证**

```bash
cd /home/boonraygrn/boonray_ws && source /opt/ros/humble/setup.bash && \
  colcon build --packages-select robominer_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
ros2 interface show robominer_msgs/msg/LocalizationData | grep -c 'invalid_reason\|zero_velocity\|static_std'   # 期望 ≥4（3字段+常量行）
```

- [ ] **Step 4: 全量重建 boonray_ws 确认下游包（flofix_loc 等）不受影响**

Expected: 全部包编译通过（msg 尾部追加，二进制兼容）
- [ ] **Step 5: Commit**（common 仓，中文提交信息写清字段语义）

### Task 3: gnss_core 判决类型定义（gate.hpp）

**Files:**
- Create: `gnss_core/include/gnss_core/gate.hpp`

**Interfaces:**
- Produces（后续所有任务依赖，签名冻结）：

```cpp
namespace gnss_core {

enum class PosType : uint8_t { None=0, Psrsp=1, Psrdiff=2, RtkFloat=3, RtkFixed=4 };
enum class InsStatus : uint8_t { InsInactive=0, InsAligning=1, InsHighVariance=2, WaitingAzimuth=3,
                                 InsSolutionFree=4, InsAlignmentComplete=5, DeterminingOrientation=6,
                                 WaitingInitialPos=7, InsSolutionGood=8, InitializingBiases=9,
                                 MotionDetect=10, Unknown=255 };
enum class RtkStatus : uint8_t { RtkInit=0, APosit=1, RtcmDiff=2, NotUsed=3, RtkFix=4, RtkFloat=5, RtkOther=6, DataTimeout=11 };
enum class FaultCode : uint8_t { None=0, InsNotConverged=1, GraceTimeout=2, HardwareWarning=3,
                                 SatelliteInsufficient=4, BaselineAbnormal=5, EnuOutOfBounds=6,
                                 ParseFail=7, DataTimeout=8, NoData=9 };
enum class InvalidReason : uint8_t { None=0, DeviceFault=1, InitNotReady=2, DiffAgeTooLarge=3,
                                     SatelliteWeak=4, SolutionAbnormal=5, Other=99 };

struct Evidence {          // 一帧判决输入；未提供的字段由适配器置缺省
  PosType pos_type = PosType::None;
  InsStatus ins_status = InsStatus::Unknown;
  double diff_age_s = 0.0;        // GGA age；未收到 GGA 保持 0（永不过期语义）
  uint16_t sats_main = 0;         // 北云=GPFPD nsv1
  uint16_t sats_aux = 0;          // 北云=HEADINGA obs
  double enu_x = 0, enu_y = 0, enu_z = 0;
  bool heading_narrow_int = false;
  double baseline_length = 0.0;
  bool hardware_fault = false;    // 华测 warning 掩码 / 北云 receiver_status（本期可不接）
  double now_s = 0.0;             // 墙钟秒，调用方注入（库内不读时钟）
};

struct GateConfig {        // 默认值=localization_parser.yaml 现值
  double xyz_threshold = 10000.0;
  uint16_t stars_threshold = 10;
  double single_point_age_threshold = 10.0;
  double heading_length = 1.6;
  double heading_length_threshold = 0.2;
  double unfixed_time = 5.0;
  double rtk_float_grace_time = 20.0;
  double sticky_grace_time = 30.0;   // 北云粘滞宽限（152519 新增）；<=0 立即判无效
};

struct Verdict { bool is_valid=false; RtkStatus rtk_status=RtkStatus::RtkInit;
                 FaultCode fault_code=FaultCode::None; InvalidReason invalid_reason=InvalidReason::None; };

struct GateState {          // 粘滞与宽限的显式状态
  bool has_prev = false;
  Verdict prev{};
  bool unfixed_timing = false;
  double unfixed_since_s = 0.0;
  bool in_sticky = false;          // 北云粘滞路径状态（152519）
  double sticky_since_s = 0.0;     // 单调钟秒
};

Verdict Judge(const Evidence& ev, const GateConfig& cfg, GateState& st);   // 北云规则
Verdict JudgeHuace(const Evidence& ev, const GateConfig& cfg, GateState& st); // 华测规则
InvalidReason MapInvalidReason(FaultCode fc);  // fault_code → 飞书 invalid_reason 映射（文档化）
}
```

- [ ] **Step 1: 写编译测试** `test/test_gate_types.cpp`：构造 Evidence/默认 GateConfig，断言默认值（stars_threshold==10 等）
- [ ] **Step 2: 运行确认失败**（头文件不存在）
- [ ] **Step 3: 写 gate.hpp**（纯类型+声明，无实现）；CMakeLists 双路径都挂上（test/ 列表参照现有 test_geodetic.cpp 的登记方式）
- [ ] **Step 4: 编译通过、类型测试 PASS**
- [ ] **Step 5: Commit** `feat(gnss_core): gate 判决类型定义（Evidence/GateState/Verdict）`

### Task 4: golden 表征捕获（北云 + 华测）

**Files:**
- Create: `localization_parser/tests/dump_golden.cpp`（挂在现有 tests 构建里，与 test_fault_code.cpp 同级）
- Create: `gnss_core/test/golden/beiyun_cases.csv` / `huace_cases.csv` / `*_expected.csv`

**Interfaces:**
- Consumes: localization_parser 现有 `BeiYunProcessor`/`HuaCeProcessor`（构造需 `transport::ITransport&`，mock 之——接口在 `include/localization_parser/core/transport/itransport.hpp`）
- Produces: golden CSV，列：`case_id, pos_type, ins_status, age, nsv1, obs, x, y, z, narrow_int, baseline, now_s → is_valid, rtk_status, fault_code`

**用例矩阵（北云 14 条，华测 8 条，每条含 frame 脚本与期望）：**
北云：① 首帧 RTKFIXED；② 首帧 FLOAT+GOOD；③ 首帧 PSRSP+!GOOD+age<10（宽限降级，is_valid 粘滞）；④ !GOOD 且非 PSRSP（InsNotConverged）；⑤ ENU 越界（EnuOutOfBounds，与 ④ 叠加时取大枚举）；⑥ nsv1<10（SatelliteInsufficient）；⑦ 无 GPFPD（nsv1=0 必触发，Review Focus #4）；⑧ narrow_int 基线 1.9m（超 1.6±0.2 → BaselineAbnormal）；⑨ 非 narrow_int 基线异常不触发；⑩ ins_status 未知字符串（粘滞上一帧，#3）；⑪ FIXED→FLOAT+GOOD→PSRSP 序列（粘滞传递）；⑫ FIXED 后再 FIXED（fault_code 清零）；⑬ **FIXED→PSRDIFF 粘滞 30s 内保持原判决**（152519 宽限内旧行为）；⑭ **粘滞超 30s → 无效 + GraceTimeout + rtk_status=RtcmDiff**（PSRSP 则 Aposit；NONE 保持上一帧）
华测：⑬ 浮点宽限 20s 内保持、⑭ 超 20s 置 GraceTimeout、⑮ 组合导航模式用 combined_nav_mod_time 档、⑯ 硬件 warning 掩码→HardwareWarning、⑰ ENU 越界、⑱ 星数不足、⑲ 首帧无数据 NoData、⑳ 粘滞序列

- [ ] **Step 1: 写 dump_golden.cpp**：mock ITransport，按用例矩阵喂帧，每个 INSPVAXA/主报文后dump一行 CSV 到 stdout
- [ ] **Step 2: 跑现行代码生成 golden**：`./dump_golden > beiyun_expected.csv`——**这一步捕获的是线上行为，不许人工修正**
- [ ] **Step 3: 人工抽查 3 条用例**对照 beiyun_processor.cpp:213-263 分支语义，确认 golden 合理（捕获错了=测试脚本错，不是线上错）
- [ ] **Step 4: Commit**（localization_parser 仓：`test: 新增 golden 表征捕获工具`；gnss_core 仓：golden CSV 入库）

### Task 5: Judge 北云规则实现（对齐 golden）

**Files:**
- Create: `gnss_core/src/gate.cpp`
- Test: `gnss_core/test/test_gate_beiyun.cpp`

**Interfaces:**
- Consumes: Task 3 类型、Task 4 golden CSV
- Produces: `Judge()` 实现，行为引用：beiyun_processor.cpp:213-263

**实现要点（分支顺序不可变）：**
1. 初值取 `st.prev`（无 prev 用 Verdict 默认）——对应 beiyun_processor.cpp:197-205
2. `RTKFIXED → valid + RtkFix + FaultCode::None`；否则 `FLOAT && ins_good → valid + RtkFloat + None`；否则 `!ins_good`：PSRSP 且 age<阈值 → 仅 rtk_status=RtkFloat（宽限降级，不动 is_valid/fault_code），否则 invalid + RtkInit + InsNotConverged
3. 硬门控依次叠加：ENU 越界（+EnuOutOfBounds）→ 星数（sats_main/sats_aux 任一 <阈值，+SatelliteInsufficient）→ narrow_int 基线 |len−1.6|>0.2（+BaselineAbnormal）；多因并存取枚举最大者（RaiseFaultCode 语义）
4. invalid_reason 由 MapInvalidReason(fault_code) 派生：InsNotConverged→InitNotReady / GraceTimeout→DiffAgeTooLarge / HardwareWarning→DeviceFault / SatelliteInsufficient→SatelliteWeak / BaselineAbnormal+EnuOutOfBounds→SolutionAbnormal / ParseFail+DataTimeout+NoData→Other / None→None
5. 粘滞宽限（beiyun_processor.cpp:239-271）：无分支命中进入粘滞——sticky_grace_time 内沿用上一帧（含 fault_code 粘滞）；超时 `is_valid=false` + RaiseFaultCode(GraceTimeout)，pos_type=PSRSP→rtk_status=Aposit、PSRDIFF→RtcmDiff、其余保持；任一确定性分支命中即退出粘滞、重新起算

- [ ] **Step 1: 写 golden 回放测试** `test_gate_beiyun.cpp`：读 CSV 逐行构造 Evidence 喂 Judge，断言 Verdict 三字段逐行相等
- [ ] **Step 2: 运行确认失败**（Judge 未实现）
- [ ] **Step 3: 实现 gate.cpp 北云路径**
- [ ] **Step 4: golden 12 条全 PASS**
- [ ] **Step 5: Commit** `feat(gnss_core): 北云判决规则迁移（golden parity）`

### Task 6: Judge 华测规则实现（对齐 golden）

**Files:**
- Modify: `gnss_core/src/gate.cpp`
- Test: `gnss_core/test/test_gate_huace.cpp`

**Interfaces:**
- Consumes: Task 3/4；行为引用 huace_processor.cpp:228-290（宽限三档、HardwareWarning 掩码、ENU 越界、粘滞）
- Produces: `JudgeHuace()`；宽限计时用 `st.unfixed_since_s` + `ev.now_s`（Review Focus #6）

- [ ] **Step 1: golden 回放测试 8 条（⑬-⑳），先失败**
- [ ] **Step 2: 实现华测路径**——宽限档位选择逻辑复刻 huace_processor.cpp:262-277
- [ ] **Step 3: 全 PASS；纯 CMake 路径（无 ament）也跑一遍** `cmake -B build -DBUILD_TESTING=ON && cmake --build build && ctest --test-dir build`
- [ ] **Step 4: Commit** `feat(gnss_core): 华测判决规则迁移（golden parity）`

### Task 7: 回放 CLI 骨架（judge_replay）

**Files:**
- Create: `gnss_core/tools/judge_replay.cpp`
- Modify: `gnss_core/CMakeLists.txt`（tools 段，参照 `add_executable(synth_rtk_fix ...)` 现有写法）

**Interfaces:**
- Consumes: `Judge/JudgeHuace`；输入为 localization_parser 落盘的原始流文件（`~/data/localization_raw/beiyun/replay/beiyun_raw_*.log`）
- Produces: CLI `judge_replay <brand> <raw.log> > verdicts.csv`——逐主报文一行 `gps_sec, is_valid, rtk_status, fault_code, invalid_reason`。**本任务只出骨架**：帧切分直接换行/ASCII 头切（北云 ASCII 报文以 `#`/`$` 起始），解析只提判决所需字段（pos_type/ins_status 等简易 sscanf），完整解析留待适配器任务（Plan 2）复用 localization_parser 的 parser 头文件

- [ ] **Step 1: 写测试**：构造 3 行样本流文件，断言输出 CSV 行数与首行字段格式
- [ ] **Step 2: 实现骨架并通过测试**
- [ ] **Step 3: 用车上现有落盘文件实跑一次**，输出行数 > 0 且 fault_code 分布可打印（人工目检合理性）
- [ ] **Step 4: Commit** `feat(gnss_core): judge_replay 回放 CLI 骨架`

---

## Self-Review 记录

- **Spec 覆盖**：本 plan 覆盖 SPEC 第 0+1 期（msg 冻结 + Judge 抽库 + parity 工具）。metrics 库、适配器节点、divergence 接入属 Plan 2（第 2 期），不在此列
- **VCU**：按决定本期不接入，Evidence 无 VCU 字段
- **类型一致性**：Task 3 冻结的枚举值与 LocalizationDataAux.msg 注释、飞书 invalid_reason 枚举逐项核对过
- **比例**：实现主体是规则迁移（golden 驱动），plan 刻意不含函数体
