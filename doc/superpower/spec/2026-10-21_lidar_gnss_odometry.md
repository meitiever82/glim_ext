# SPEC：LiDAR+GNSS 紧耦合定位生产线（替代 flofix_loc）

> 日期：2026-10-21（v2，推翻 v1 的"新起包"路线）
> 状态：草稿，待用户 review。**带 ⚠️ 的条目是未经确认的假设/待拍板项**
> 交付基座：**glim_ws**（glim_underground + glim_ros2 + glim_ext + gnss_core + gnss_bringup）
> 目标测试平台：**geacx 小车**，以 `finder_ros/drivers/sensor_launcher/scripts/start_boonray_all_and_save.sh`（2026-10-12 版）描述的传感器栈为准；未来新增车辆 = 新增对应 sh + glim 配置集
> 部署目标：湖州 20 台车
> 原型依据：紧耦合原型（锁外参 ATE 0.100m，当前全场最强前端）

## Problem Statement

flofix_loc 在湖州项目现场不稳定、定位飘。根因是架构：它用 RTK FLOAT/FIX 代理室内/室外，靠"4 阶段状态机 + 综导主权切换"工作，fix#1~13 全部在打切换沿的信任补丁（假 FIX 跑飞 7.76m、多径 seed 污染、跳变停稳型假 Fix 无限慢拖……），每个补丁又引入新误伤面。20 台车规模下"补丁堆信任门"不可维护。

glim_ws 一侧的既有实证：
- 紧耦合原型（IMU 预积分 + GNSS 位置因子 + KISS between，锁外参）ATE **0.100m**，优于调研松耦合最优 0.155、GLIM+CT+RTK（rtk_odometry 外层注入）0.325
- 一期结论明确了**剩余差距即架构差距**：rtk_odometry 因子进 CT 里程计的**外层**固定滞后平滑器，外层被先验钉在内层 LM 结果上，前端内层一发散外层 RTK 救不回来——要兜底必须注入内层 LM（一期就标注为"二期设计议题"）
- 二期调研同时证伪了在线外参平移估计（与 INS 内部天线杆臂同构多模态），外参维持离线锁定

**既定目标**：在 glim_ws 架构上把紧耦合从离线原型变成车端全程在线的 LiDAR+IMU+GNSS 里程计——GNSS 全程进图、无 FLOAT/FIX 状态机、无主权切换，替代 flofix_loc。

## Solution

### 核心：紧耦合 odometry 模块（内层注入）

在 GLIM 的 odometry_estimation 族中新增紧耦合实现（`odometry_estimation_tight`，⚠️ 命名待定），把现有"外层固定滞后贴 RTK"改为**内层因子图**：

- **状态**：`X(k)` 位姿、`V(k)` 速度、`B(k)` IMU 零偏（GTSAM iSAM2 增量）
- **IMU 预积分因子**：帧间主干，GNSS 中断期递推能力来源——这是 flofix_loc DR 桥接、coast 锁死等补丁的架构性替代
- **激光 between 因子**：scan-to-scan 相对约束进**内层**（原型血缘：KISS）；与 GLIM 现有 CT 前端的关系（复用 CT 的 deskew/体素管线，还是移植 KISS）⚠️ plan 阶段定
- **GNSS 位置因子**：北云**原始 RTK 位置**经标定杆臂入图，**不用 INSPVA 融合位置**（避杆臂同构混淆——二期实证根因）；定权 = 板卡 σ + robust kernel，σ_floor 留系统误差量级（一期实证规则）。湖州车型传感器栈与 geacx 同类，**无需新增驱动**（用户已确认）
- **外参**：T_lidar_imu / 天线杆臂全部离线标定锁定（二期实证：平移在线不可观）
- **零速因子**（可选）：静止段 ZUPT；VCU 轮速数据质量未验证，先不接

### 无状态机的机制对应

| flofix_loc 行为 | 本系统对应机制 |
|---|---|
| 阶段1 库内纯激光 LO | GNSS σ 大/缺失 → 因子自然失效，IMU+LIO 递推 |
| 阶段2 coast / 阶段3/4 软追硬贴 | 不存在——GNSS 全程在图内按 σ 定权 |
| fix#7 DR 桥接 / fix#10/12/13 假 FIX 防护 | IMU 预积分桥接 + 鲁棒核/新息门控，非状态锁存 |
| RateLifter 50Hz vcu+wz 外推 | IMU 传播，天然 ≥100Hz |

### 平台适配：geacx 小车（首平台）

以 `start_boonray_all_and_save.sh` + `boonray_smallcar.launch.py` 为准的现实：

| 资源 | 现状 | 本系统的用法 |
|---|---|---|
| ROS2 Humble（Jetson） | ✅ | 无需 ROS1 适配层 |
| 北云 INS 192.168.7.110 | ✅ bynav_ros_driver | IMU=`/imu/data_raw`；GNSS=RtkFix（gnss_bringup 已部署 geacx，含 rtkrcv 独立解算链路） |
| RoboSense Helios ×3 | ✅ `/rslidar_*` | 本期不入图（GLIM 单雷达架构，见下） |
| **主雷达：Seyond K2（前雷达）** | ✅ 默认启用 | **唯一定位雷达**（用户已拍板）。注意 K2 无 IMU——IMU 全部由北云 `/imu/data_raw` 供给 |
| Livox Avia | 与 K2 互斥，默认关 | 不用 |
| GMSL 相机 ×4 | ✅ | 本期不用（纯 LiDAR+GNSS） |
| PTP 时间同步 | ✅ timesync_infra | 紧耦合对时标敏感，依赖其正确性；验收含时标对齐检查 |
| 算力 | Jetson Orin ⚠️ 具体型号与余量待实测 | 原型为离线批量 LM；在线 iSAM2 + 前端在 Orin 上的实时性是**首要技术风险**，plan 必须有算力预算实测任务 |

新增车辆（含湖州车型）= 新增 `start_<车>_and_save.sh` + `glim_underground/config/<vehicle>/` 配置集 + 杆臂/外参离线标定，不动框架代码。

### 与 gnss_bringup 重构线的关系

并行不冲突：gnss_bringup 线交付门控判决/消息/指标（LocalizationData 由谁发），本线交付位姿估计。geacx 上 gnss_bringup 已部署（691786f），RtkFix 输入链路现成。两边在"最终 LocalizationData 内容"处收口对齐 ⚠️。

### 架构分层（沿用既有纪律）

```
glim_ros2 适配层        订阅 /rslidar_*、/imu/data_raw、RtkFix → GLIM 帧
glim（core，无 ROS）    新 odometry_estimation_tight（iSAM2 内层）+ sub/global mapping 沿用
glim_ext modules        rtk_global/rtk_odometry 保留作 A/B 对照；新紧耦合因子实现
gnss_core（复用）       geodetic / pos_io / RtkFix / 杆臂工具
```

约束沿用：core 禁 ROS；确定性平台（num_threads=1）做 A/B；<0.3m 单跑 delta 在非确定平台上不许报结论。

## User Stories

1. 作为规控模块，定位输出任何工况（棚内/棚外/多径/GNSS 切换沿）不跳变，C0 连续
2. 作为现场工程师，判断定位健康不需要理解 FLOAT/FIX 状态机——诊断读前端质量/IMU/GNSS 新息三类指标
3. 作为算法工程师，GNSS 质量变化是定权问题（σ+鲁棒核），行为可用表单化单测覆盖
4. 作为项目，新车型接入 = 加 sh + 加配置集 + 离线标定，不改代码
5. 作为项目，湖州 20 台替换时与 flofix_loc 影子双跑、可逐车回退

## Implementation Decisions

- **基座血缘**：因子与预积分实现以 tck_calib_v2 原型为种子；集成进 GLIM odometry 流水线而非独立进程——直接消除"外层救不了内层"的一期架构限制
- **优化器**：GTSAM iSAM2 增量（原型是批量 LM）；glim 本机 GTSAM 4.3a1 ⚠️ geacx 车端 `~/gtsam-local` 已有 4.3a1（启动需 LD_LIBRARY_PATH 优先于 /usr/local 裸拷 4.2.0——部署文档既有教训）
- **前端选择**：**KISS（用户已拍板）**——原型血缘，0.100m 数字可直接作为验收基线；需移植进 GLIM odometry 流水线（deskew/体素管线复用范围在 plan 阶段定）
- **接口**：输出对齐规控消费的定位话题 ⚠️（geacx 上现行为 bynav/综导直发？需查 boonray_smallcar launch 下游——plan 任务）
- **is_valid**：由图健康度表达（位姿协方差 + GNSS 新息 + 前端配准质量），判据 plan 阶段定义
- **杆臂/外参**：geacx 需做一次离线标定（gnss_core estimate_lever_arm + lidar_calibrate 既有工具链）

## Testing Decisions

- **因子数学单测**：自定义因子先写独立 LM 小程序验残差/雅可比（glim 传统）
- **验收门槛（硬性）**：湖州现场"定位飘"bag 回放——飘的场景必须全部消除且不引入回归；另以既有验证数据集 seg A 对 gt ATE ≤ **0.100m**（原型数字）作精度基线；不得以"能跑"交差
- **geacx 实车**：户外场地 RTK FIXED 段 ATE + 棚/遮挡段递推漂移率；与 flofix 问题形态（跳变/慢拖/假 FIX）逐一对照验证"架构上不存在"
- **湖州 bag**：有现场 bag（拿不到就去现场录，不麻烦）——验收主基准；部署后新问题形态回录补充回归集
- **确定性纪律**：A/B 一律 num_threads=1 平台
- **双跑切换**：湖州替换影子双跑 → 逐车切换 → flofix_loc 只读

## Out of Scope

- 相机/GMSL 任何用途（本期纯 LiDAR+IMU+GNSS）
- 在线外参平移估计（已结构性证伪）；旋转在线为后续可选
- 建图/回环：**湖州一期只交付 Odometry 层**。回环在架构上归 GLIM GlobalMapping 层（既有设施，submap 级位姿图），一期不启用，列二期任务
- 云端指标可视化（gnss_bringup 线 scope）
- VCU 轮速因子、610 杆臂标定（geacx 是北云，不涉及 610）

## 已拍板（2026-10-21 用户确认）

| # | 项 | 结论 |
|---|---|---|
| Q1 | 湖州车传感器栈 | 与 geacx 同类，**无需新增驱动** |
| Q2 | 主雷达 | **Seyond K2（前雷达）**；Helios×3 本期不入图；K2 无 IMU，IMU 全由北云供 |
| Q3 | 湖州飘场景 bag | **有**（拿不到就去现场录）——验收主基准 |
| Q4 | 前端血缘 | **KISS** |

## 待拍板（Review 时逐条过）

| # | 开口 | 建议默认值 |
|---|---|---|
| Q5 | 输出话题与 is_valid 消费者现状（geacx 下游谁订阅定位） | plan 任务：先查 boonray_smallcar 下游拓扑再定 |
| Q6 | geacx Orin 算力预算（前端+iSAM2 在线实时性） | plan 的第一个实测任务；不达标则先砍 sub/global mapping 只留 odometry |
| Q7 | 交付节奏：去湖州前要哪一步 | 建议：geacx 上全程在线跑通 + 湖州 bag 回放达标 |
