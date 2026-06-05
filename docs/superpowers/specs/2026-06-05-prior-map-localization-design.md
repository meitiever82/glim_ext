# 设计:`prior_map_localization` —— 基于先验地图的紧耦合定位 glim_ext 模块

| 项目 | 内容 |
|------|------|
| 模块名 | `prior_map_localization` |
| 产物 | `libprior_map_localization.so`(glim_ext 扩展) |
| 版本 | MVP (v1) |
| 日期 | 2026-06-05 |
| 理论依据 | ROBOMECH 2023《Window-Optimization-based Range-IMU Localization on a 3D Prior Environmental Map》(`glim-underground/docs/glim/robomech2023.pdf` 及译文) |
| 关联文档 | `glim-underground/docs/glim/glim_localization_proposal.md` |

---

## 1. 背景与目标

### 1.1 问题

在先验地图上做 LiDAR-IMU 定位,现有两类做法各有短板:

- **纯地图匹配(FAST_LIO_LOCALIZATION 式)**:每帧直接配准到先验地图,地图质量差/环境变化时匹配失败、滤波发散。
- **松耦合切换(本仓库现有 `glim_localization` 节点,在 `casbot_ws`/`rtabmap_ws`)**:GLIM 当纯里程计跑,独立节点周期性把当前 scan 配准到 `/globalmap`,发 `map→odom` 修正。切换时刻不连续,两类约束无法联合优化。

ROBOMECH2023 提出的紧耦合做法:在**同一个滑动窗口因子图**里同时融合 Scan-to-Scan(S2S)、Scan-to-Prior-Map(S2M)、IMU 预积分三类约束;用**完全相同的误差函数**处理 S2S 与 S2M,使得地图缺失区域自动平滑退化为纯 LIO,回到地图范围后再次收敛。

### 1.2 本模块目标(MVP)

基于 GLIM 的全局回调插槽,**零侵入 GLIM 核心**,实现 ROBOMECH2023 的紧耦合定位核心闭环:

1. 加载 PCD 先验地图,构建 GPU/CPU `GaussianVoxelMap`。
2. 通过 `OdometryEstimationCallbacks::on_smoother_update` 向 GLIM 滑动窗口因子图注入 S2M(VGICP)因子。
3. 基于 scan 与地图的**重叠率**自适应调整 S2M 因子权重;重叠率过低则不注入因子 → 自动退化为纯 LIO。
4. 支持 RViz `/initialpose` 与 config 默认位姿两种初始定位。
5. 发布 `map→odom` TF 及定位状态话题。

### 1.3 非目标(留待 v2)

- 自定义退化感知因子、Hessian 特征值退化检测与按方向降权。
- FPFH+RANSAC 全局重定位、GNSS 初始化。
- `LOST` 状态机与自动重定位、`/localization/relocalize` 等服务。
- GLIM dump 目录 / `/globalmap` 话题加载、大地图分块加载。

---

## 2. 关键技术决策

### 2.1 复用 `gtsam_points::IntegratedVGICPFactor`,不自写 S2M 因子

`gtsam_points 1.2.0` 已提供「固定目标位姿」重载,等价于 proposal 中计划自写的 `ScanToPriorMapFactor`,且有 GPU 版:

```cpp
// gtsam_points/factors/integrated_vgicp_factor.hpp
IntegratedVGICPFactor_(
    const gtsam::Pose3& fixed_target_pose,                 // 先验地图在 odom 系中的位姿 = T_odom_map
    gtsam::Key source_key,                                 // X(i),即 T_odom_imu(i)
    const GaussianVoxelMap::ConstPtr& target_voxels,       // 先验地图体素图
    const std::shared_ptr<const SourceFrame>& source);     // 当前帧去畸变点云
```

这是一个 `NoiseModelFactor1<Pose3>`,只优化 source 位姿,target 固定。`EstimationFrame::frame`(已去畸变、sensor 系点云)直接作为 source。**MVP 因此不写任何自定义因子**,把最易出错的退化检测整块推迟到 v2。

`use_gpu=true` 时用 `IntegratedVGICPFactorGPU`,否则用 CPU 版;与 GLIM 自身 CUDA 开关对齐。

### 2.2 坐标系策略:`map→odom` 发常量(做法 A,GLIL 式)

GLIM 里程计在自己的 `odom` world 系跑(重力对齐、原点≈起点)。不修改 GLIM 的初始位姿设定(它未向扩展暴露)。而是:

- 初始定位给出 `T_map_imu(0)` → `T_map_odom = T_map_imu(0) · X(0)⁻¹`(X(0)≈identity,故 `T_map_odom ≈ 初始位姿`)。
- S2M 因子的 `fixed_target_pose = T_odom_map = T_map_odom⁻¹`。优化时 S2M 把 `X(i)` 拉向与地图一致,S2S+IMU 提供相对运动兜底。
- **漂移修正被吸收进 GLIM 优化输出的 `X(i)`**;`T_map_odom` 作为常量发布。

残差几何验证:因子最小化 `X(i)·p_source` 与 `T_odom_map·voxel` 的对齐;代入 `T_odom_map = T_map_odom⁻¹` 得 `T_map_odom·X(i)·p_source ≈ map_point`,即 scan 落到地图上。✓

> 备选(不采用):打 GLIM 核心补丁,seed 第一帧 `T_world_imu = 初始位姿` 使 world≡map、因子目标用 identity。概念更干净但侵入 `NaiveInitialStateEstimation::set_init_state` / 需加 core 配置。MVP 不做。

> 对下游导航的影响:做法 A 下 `odom→base` 已被地图约束影响,非"纯漂移里程计"。若未来某下游强依赖 `odom→base` 严格连续,再在 v2 引入做法 B(GLIM 输出保持纯漂移、每帧反算并平滑 `T_map_odom`)。

---

## 3. 系统架构

### 3.1 模块形态与位置

- 新建 `src/glim_ext/modules/localization/prior_map_localization/`,编为 `libprior_map_localization.so`。
- 入口 `extern "C" glim::ExtensionModule* create_extension_module()`,返回继承 `glim::ExtensionModule` 的实例;构造函数内注册回调(参照 `modules/odometry/velocity_suppressor`)。
- `glim_ext/CMakeLists.txt` 增加 `option(ENABLE_PRIOR_MAP_LOC "Enable prior-map localization" ON)` + `add_subdirectory(modules/localization/prior_map_localization)`。
- 启用:把 `libprior_map_localization.so` 加入 `glim-underground/config/config_ros.json` 的 `extension_modules`。

### 3.2 组件划分(单一职责,可独立测试)

```
prior_map_localization/
├── include/glim_ext/
│   ├── prior_map_manager.hpp        # 地图加载 + 体素图 + 重叠率
│   ├── initial_pose_source.hpp      # /initialpose + config 默认位姿
│   └── prior_map_localization.hpp   # ExtensionModule 主类
├── src/glim_ext/
│   ├── prior_map_manager.cpp
│   ├── initial_pose_source.cpp
│   └── prior_map_localization.cpp   # 含 create_extension_module()
└── CMakeLists.txt
```

| 组件 | 职责 | 依赖 | 可测性 |
|------|------|------|--------|
| `PriorMapManager` | 加载 PCD → 构建 `GaussianVoxelMap`;`voxelmap()`、`is_in_bounds(p)`、`compute_overlap_ratio(cloud, T_map_sensor)` | gtsam_points(无 ROS) | 纯单测 |
| `InitialPoseSource` | 订阅 `/initialpose`,持有 config 默认位姿;线程安全 `pop_pending()` | rclcpp | 逻辑单测 |
| `PriorMapLocalization` | `ExtensionModule`;注册 `on_new_frame`(算重叠率、缓存权重)与 `on_smoother_update`(注入 S2M 因子);维护 `T_odom_map` 与 `state`;发布 TF/话题 | glim, gtsam_points, rclcpp | 集成测 |

### 3.3 状态(MVP 仅两态)

- `UNINITIALIZED`:等待初始位姿。收到 `/initialpose` 或应用 config 默认位姿后,设定 `T_odom_map` 初值 → `TRACKING`。
- `TRACKING`:正常注入 S2M 因子。
- (`INITIALIZING` / `LOST` / 自动重定位 → v2。)

---

## 4. 数据流(每帧)

```
/initialpose ─┐
              ▼
   InitialPoseSource ──(首帧/重定位请求)──► 设定 T_odom_map 初值, state=TRACKING

GLIM odometry ──on_new_frame(frame)──►
    若 state==TRACKING:
      T_map_sensor = T_map_odom · frame->T_world_imu
      overlap = PriorMapManager.compute_overlap_ratio(frame->frame, T_map_sensor)
      weight  = adaptive_weight(overlap)            // 见 §5.1
      缓存 { frame->id : weight }

GLIM smoother ──on_smoother_update(smoother, new_factors, new_values, new_stamps)──►
    对 new_values 中的新帧 key X(id)(仅窗口内变量):
      若 state==TRACKING 且 cached_weight(id) > 0:
        noise   = base_noise / sqrt(weight)         // 权重越低噪声越大
        factor  = IntegratedVGICPFactor(T_odom_map, X(id), prior_voxelmap, frame->frame, noise)
        new_factors.add(factor)
      // weight==0(重叠率过低)→ 不注入 → 该帧纯 LIO

(优化由 GLIM 完成;做法 A 下 T_map_odom 为常量,无需反算)
    ▼
发布:map→odom TF(常量 T_map_odom)
      /localization/pose  (PoseStamped, = T_map_odom · T_odom_base)
      /localization/overlap (Float64)
      /localization/status  (String: UNINITIALIZED/TRACKING)
```

**窗口约束**:S2M 因子只对 `on_smoother_update` 的 `new_values` 中出现的新帧 key 注入,绝不引用已边缘化变量,满足 GLIM「附加因子只能引用优化窗口内变量」的硬约束。

---

## 5. 关键算法

### 5.1 自适应因子权重(沿用 proposal §5.1)

```
adaptive_weight(overlap):
    if overlap < min_overlap:            return 0.0          # 不注入因子,纯 LIO
    if overlap >= full_weight_overlap:   return 1.0          # 全权重
    return (overlap - min_overlap) / (full_weight_overlap - min_overlap)   # 线性
```

权重映射到噪声模型:`noise = base_noise / sqrt(weight)`(权重越低,S2M 约束越弱)。`base_noise` 由 config 的 `s2m_noise_rot`(rad)/`s2m_noise_trans`(m)给出,构成 6 维各向同性对角噪声。

### 5.2 重叠率计算

`compute_overlap_ratio(cloud, T_map_sensor)`:对 cloud 每点经 `T_map_sensor` 变换后,查先验地图 voxelmap 是否命中(或近邻距离 < `max_corr_dist`),命中点数 / 总点数。MVP 用 voxelmap 体素命中判定即可,无需额外 KdTree。

---

## 6. 配置与 ROS 接口(MVP 最小集)

### 6.1 配置文件

`glim_ext/config/config_prior_map_localization.json`(走 GLIM config 寻址回退机制:先查主 `config.json`,再回退 glim_ext):

```jsonc
{
  "prior_map_localization": {
    "prior_map_path": "/path/to/prior_map.pcd",
    "voxel_resolution": 0.5,
    "use_gpu": true,
    "min_overlap": 0.05,
    "full_weight_overlap": 0.30,
    "s2m_noise_rot": 0.1,          // rad
    "s2m_noise_trans": 0.2,        // m
    "max_corr_dist": 1.0,          // m,重叠率命中阈值
    "default_init_pose": [0,0,0,0] // x y z yaw(map 系);留空则等待 /initialpose
  }
}
```

### 6.2 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz 2D Pose Estimate |
| 发布(TF) | `map → odom` | `tf2` | 常量 `T_map_odom` |
| 发布 | `/localization/pose` | `geometry_msgs/PoseStamped` | `T_map_odom · T_odom_base` |
| 发布 | `/localization/overlap` | `std_msgs/Float64` | 当前帧重叠率 |
| 发布 | `/localization/status` | `std_msgs/String` | `UNINITIALIZED` / `TRACKING` |

服务、FPFH、GNSS、`LOST`/自动重定位 → v2。

---

## 7. 测试策略

### 7.1 单元测试(脱 ROS)

| 测试项 | 内容 | 通过标准 |
|--------|------|----------|
| 地图加载 | 加载已知 PCD | 点数与文件一致 |
| 重叠率 | 构造全重叠 / 零重叠场景 | 误差 < 5% |
| VGICP 因子 | 给定位姿下因子残差/雅可比 | 与 `gtsam::numericalDerivative` 数值梯度一致 |
| 自适应权重 | overlap 跨 min/full 边界 | 分段映射正确、边界连续 |

### 7.2 集成测试

| 场景 | 内容 | 通过标准 |
|------|------|----------|
| 初始定位 | 发 `/initialpose` 后 | `map→odom` 收敛,轨迹贴合地图 |
| 地图内全程 | 全程在地图范围 | APE < 0.1 m |
| 地图缺失区域 | 走出地图 → 返回 | 重叠率→0、因子停注、不漂飞;回到地图后重新收敛(论文核心卖点) |

### 7.3 性能

| 指标 | 目标 |
|------|------|
| 单帧 S2M 注入+优化增量 | < 100 ms(GPU) |
| GPU 显存 | < 2 GB(典型地图) |

---

## 8. 风险与对策

| 风险 | 对策 |
|------|------|
| 做法 A 下 `odom→base` 非纯漂移,下游导航可能不适应 | MVP 暂不处理;必要时 v2 引入做法 B |
| 大 PCD 一次性加载显存不足 | MVP 限定中小地图;分块加载留 v2 |
| 初始位姿误差大导致首帧 S2M 误匹配 | MVP 要求 `/initialpose` 足够准;FPFH 全局初始化留 v2 |
| GLIM 窗口边缘化时序导致因子引用越界 | 严格只对 `new_values` 中 key 注入,集成测覆盖 |

---

## 9. 参考

- ROBOMECH2023 译文:`glim-underground/docs/glim/robomech2023_译文.md`
- 原始 proposal(完整版,含 v2 内容):`glim-underground/docs/glim/glim_localization_proposal.md`
- 因子注入参考实现:`glim_ext/modules/odometry/velocity_suppressor/`
- GLIM 回调 API:`glim-underground/include/glim/odometry/callbacks.hpp`
- VGICP 固定目标因子:`gtsam_points/factors/integrated_vgicp_factor.hpp`
