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

1. 加载 PCD 先验地图,构建 CPU `GaussianVoxelMap`(GPU 加速留 v2,见 §2.1)。
2. 通过 `OdometryEstimationCallbacks::on_smoother_update` 向 GLIM 滑动窗口因子图注入 S2M(VGICP)因子。
3. 基于 scan 与地图的**重叠率**二值决定是否注入 S2M 因子;重叠率过低则不注入 → 自动退化为纯 LIO(连续降权见 §2.3,留 v2)。
4. 支持 RViz `/initialpose`(2D)与 config 默认位姿两种初始定位,并把 2D 输入补成 6DoF(z 从地图查询、roll/pitch 取自 IMU 姿态),支持中途重定位(见 §2.4)。
5. 发布 `map→odom` TF 及定位状态话题。

### 1.3 非目标(留待 v2)

- 自定义退化感知因子、Hessian 特征值退化检测与按方向降权。
- FPFH+RANSAC 全局重定位、GNSS 初始化。
- `LOST` 状态机与自动重定位、`/localization/relocalize` 等服务。
- GLIM dump 目录 / `/globalmap` 话题加载、大地图分块加载。

---

## 2. 关键技术决策

### 2.1 复用 `gtsam_points::IntegratedVGICPFactor`,不自写 S2M 因子

`gtsam_points 1.2.0` 已提供「固定目标位姿」重载,等价于 proposal 中计划自写的 `ScanToPriorMapFactor`:

```cpp
// gtsam_points/factors/integrated_vgicp_factor.hpp
IntegratedVGICPFactor_(
    const gtsam::Pose3& fixed_target_pose,                 // 先验地图在 odom 系中的位姿 = T_odom_map
    gtsam::Key source_key,                                 // X(i),即 T_odom_imu(i)
    const GaussianVoxelMap::ConstPtr& target_voxels,       // 先验地图体素图
    const std::shared_ptr<const SourceFrame>& source);     // 当前帧去畸变点云
```

只优化 source 位姿,target 固定。`EstimationFrame::frame`(已去畸变、sensor 系点云)直接作为 source。**MVP 不写任何自定义因子**,退化检测整块推迟到 v2。

**类层次(已查 gtsam_points 1.2.0 源码核实,与 proposal 的描述不同)**:

- CPU 版 `IntegratedVGICPFactor_ : IntegratedMatchingCostFactor : gtsam::NonlinearFactor` —— **不是** `NoiseModelFactor1`。直接 override `error()` / `linearize()`,误差是逐点 Mahalanobis 距离之和,由体素+点协方差决定。
- **构造函数不接收任何 `SharedNoiseModel`**;因子也没有 robust kernel 或 cost-scale setter(仅 `set_num_threads`、`set_fused_cov_cache_mode`)。→ 这直接决定了自适应权重的实现方式,见 §2.3。
- 因子自带 `inlier_fraction()` / `num_inliers()`(source 点命中体素的比例),可作为运行时重叠率监控量。
- GPU 版 `IntegratedVGICPFactorGPU : NonlinearFactorGPU`(基类不同),由 gtsam_points 的 GPU 批量线性化机制处理。

**MVP 的 S2M 因子统一用 CPU 版 `IntegratedVGICPFactor`**,即使 GLIM 里程计跑在 GPU 模式:每帧仅注入一个 S2M 因子,CPU 开销可控,且作为普通 `NonlinearFactor` 可被任意 smoother 接管;由此绕开「GPU 因子注入 GPU smoother 是否被正确接管」这一未验证的集成风险。先验地图相应构建为 **CPU `GaussianVoxelMap`**。`use_gpu` 加速 S2M 降级为 v2 优化项。

### 2.2 坐标系策略:`map→odom` 发常量(做法 A,GLIL 式)

GLIM 里程计在自己的 `odom` world 系跑(重力对齐、原点≈起点)。不修改 GLIM 的初始位姿设定(它未向扩展暴露)。而是:

- 初始定位在时刻 `t` 给出 `T_map_imu(t)`(6DoF 完整位姿,构造见 §2.4)→ `T_map_odom = T_map_imu(t) · X(t)⁻¹`,其中 `X(t)` 是**应用初始位姿那一刻**的实际里程计位姿 `T_odom_imu(t)`。
  - ⚠️ **不可假设 `X(t)=identity`**:启动时 GLIM 重力初始化后 `X(0)` 未必精确为 identity;中途重定位(运行中再发 `/initialpose`)时 `X(t)` 远非 identity。实现必须从 `on_new_frame` 缓存的最新帧取 `X(t)`,再据上式解算 `T_map_odom`。
- S2M 因子的 `fixed_target_pose = T_odom_map = T_map_odom⁻¹`。优化时 S2M 把 `X(i)` 拉向与地图一致,S2S+IMU 提供相对运动兜底。
- **漂移修正被吸收进 GLIM 优化输出的 `X(i)`**;`T_map_odom` 作为常量发布(重定位时一次性跳变到新值)。

残差几何验证:因子最小化 `X(i)·p_source` 与 `T_odom_map·voxel` 的对齐;代入 `T_odom_map = T_map_odom⁻¹` 得 `T_map_odom·X(i)·p_source ≈ map_point`,即 scan 落到地图上。✓

> 备选(不采用):打 GLIM 核心补丁,seed 第一帧 `T_world_imu = 初始位姿` 使 world≡map、因子目标用 identity。概念更干净但侵入 `NaiveInitialStateEstimation::set_init_state` / 需加 core 配置。MVP 不做。

> 对下游导航的影响:做法 A 下 `odom→base` 已被地图约束影响,非"纯漂移里程计"。若未来某下游强依赖 `odom→base` 严格连续,再在 v2 引入做法 B(GLIM 输出保持纯漂移、每帧反算并平滑 `T_map_odom`)。

### 2.3 权重机制:二值注入决策 + 子采样强度旋钮

因为 §2.1 已确认该因子**没有噪声模型、无 robust kernel、无 cost-scale setter**,proposal/原设计里「`noise = base_noise/sqrt(weight)` 构成 6 维对角噪声」的路径**不成立**。但 VGICP 代价是**逐点 Mahalanobis 求和**,所以 source 点数本身就是一个线性强度旋钮。重叠率机制改为两层:

1. **每帧注入决策(二值)**:`overlap ≥ min_overlap` 则注入 S2M 因子,否则不注入(纯 LIO)。论文核心行为(地图内 S2M 生效;走出地图 → 退化为 LIO)完整保留。CPU/GPU 因子通用。
2. **S2M 全局强度旋钮(`s2m_points_ratio`,默认 1.0)**:注入前对 `frame->frame` 做**比例随机子采样**,点数线性缩放 S2M 相对 S2S+IMU 的信息量。这是零代码的粗调旋钮,集成测若发现 S2M 过强/过弱先调它。
   - ⚠️ **必须随机抽样,不可按索引截断**:RoboSense Airy 等非重复扫描的点索引与空间分布强相关,截断会引入空间偏置。
   - 副作用利好:点数减少同时降低每次 `linearize` 的 CPU 开销(ISAM2 重线性化会多次调用该因子);注入时一并 `set_num_threads()`。

> **v2 连续降权**:若需逐帧按重叠率连续加权,最廉价的路径就是把 `s2m_points_ratio` 做成 `overlap` 的函数(抽样比例随重叠率变化),无需自定义因子。仅当需要严格信息矩阵缩放时,才退而写 CPU `NonlinearFactor` wrapper(对内层因子 `linearize()` 结果的白化 Jacobian 乘 `sqrt(weight)`)—— **注意该 wrapper 与 GPU 因子不兼容**。MVP 不实现连续降权。

### 2.4 初始位姿构造:2D `/initialpose` → 6DoF `T_map_imu(t)`

RViz `/initialpose` 只给 **x、y、yaw**(z=0、roll=pitch=0)。井下有坡度时,直接用 z=0、roll=pitch=0 会把初值踢出 VGICP 收敛域,导致首帧 S2M 误匹配或不收敛。MVP 必须把 2D 输入补成 6DoF:

| 自由度 | 来源 | 说明 |
|--------|------|------|
| x, y | `/initialpose`(或 config `default_init_pose`) | 平面位置 |
| yaw | `/initialpose` | 平面朝向 |
| **z** | **从先验地图 2D 高度格网查询 (x,y) 地面高度** | `PriorMapManager::query_ground_height(x, y)` 查预计算的粗格网(见下)。坡度场景下这是关键修正。 |
| **roll, pitch** | **取自当时帧 `X(t)` 的重力对齐姿态** | `odom` 与 `map` 共重力方向,故 IMU 估计的 roll/pitch 在 map 系同样有效;不取 0。 |

**2D 高度格网(替代保留原始点云,控内存)**:加载 PCD 时,按 `ground_grid_resolution`(如 1m)的 (x,y) 格子聚合,每格存该格内点的低位高度(最低簇 z 中位数,抗离群),建成一张 `map<(ix,iy), z>`;随后**释放原始全局点云**。查询 O(1),内存从 GB 级(整云)降到 MB 级。对"把初值拉进 VGICP 收敛域"而言,1m 精度足够。

构造步骤(在时刻 `t` 收到 `/initialpose` 时):
1. `R_rp` ← 从 `X(t).rotation` 提取 roll/pitch(去掉其 yaw 分量)。
2. `z` ← `query_ground_height(x, y)` + `imu_height_above_ground`(config,IMU 相对地面的标称高度;缺省 0)。
3. `T_map_imu(t)` ← `Translation(x, y, z) · Rz(yaw) · R_rp`。
4. (可选,默认开)**一次性精配准**:以 `T_map_imu(t)` 为初值,用当前帧点云对先验地图跑一次独立 VGICP 优化(复用 `IntegratedVGICPFactor` 建一个最小单变量图),把初值拉进收敛域,得精化后的 `T_map_imu(t)`。由 config `refine_initial_pose` 控制。
5. `T_map_odom = T_map_imu(t) · X(t)⁻¹`(见 §2.2)。
6. **冷却期(关键正确性约束,仅中途重定位)**:在 `TRACKING` 态再次收到 `/initialpose`(真正的中途重定位)使 `T_map_odom` 跳变后,窗口内**已注入的旧 S2M 因子仍持有旧的 `fixed_target_pose`**,会把窗口内历史状态往旧对齐拉,与新因子/新 `T_map_odom` 自相矛盾。GLIM `on_smoother_update` **不暴露因子删除接口**,故重定位后**冷却 `effective_cooldown`,期间不注入任何 S2M 因子**(纯 LIO 兜底,几秒漂移可忽略);待旧因子全部边缘化出窗后再恢复注入。
   - **首帧初始化(`UNINITIALIZED → TRACKING`)不进冷却**:窗口本就空、无旧因子,应立即开始注入 S2M;若也设冷却会让系统初始化后纯 LIO 空跑数秒、白白延迟地图约束生效。冷却**仅**在 `TRACKING → TRACKING` 重定位路径启用。
   - **冷却时长是正确性约束,非调参**:目的就是等旧因子全部边缘化出窗,故必须 `effective_cooldown ≥ GLIM smoother_lag`。实现:启动时读 GLIM odometry 配置的 `smoother_lag`,`effective_cooldown = max(relocalization_cooldown_sec, smoother_lag + 余量)`;若配置值偏小则自动抬高并告警。读不到 lag 时退到保守缺省并告警。
   - 实现:重定位时记 `cooldown_until_stamp = t + effective_cooldown`;`on_new_frame` 注入前检查 `frame->stamp ≥ cooldown_until_stamp`。

> 若先验地图在 (x,y) 处无点(查询失败),回退为 z=`default_init_pose.z`,并发警告日志 —— 提示用户初值落在地图外。

> **地图坐标系前提**:§2.4 的 roll/pitch 取自 `X(t)`、z 查询假设 `map` 系 **z 轴与重力对齐(z-up)**。GLIM 自建地图天然满足;若 `prior_map_path` 指向经后处理的 PCD(CloudCompare 手动旋转、他工具配准等),此前提可能被破坏,§2.4 整节失效。**MVP:由 `assume_gravity_aligned: true` 显式声明担责,加载时仅做告警级"地面存在性"启发检查,不拒启。**
> ⚠️ **不可用"主平面法向"判定**:井下巷道侧壁点远多于地面点(两面墙 vs 一条底板),最大平面很可能是墙、法向近水平 → 正确的 z-up 地图被误判拒启,恰好打在本项目场景上。
> MVP 检查改为**地面存在性**:看是否存在"法向 ≈ ±z 的大平面"(等价:对点云 z 直方图找到一条占比显著的低位地面带,或抽样点局部法向中有足够比例接近 z 轴)。**失败只告警、不拒启**。严格校验(RANSAC 按法向分组拟合地平面)留 v2。见 §6.1 `assume_gravity_aligned`。

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
| `PriorMapManager` | 加载 PCD → 构建 CPU `GaussianVoxelMap` + **粗分辨率 2D 高度格网**,随后释放原始点云;`voxelmap()`、`is_in_bounds(p)`、`compute_overlap_ratio(cloud, T_map_sensor)`、`query_ground_height(x, y)`(查格网,O(1)) | gtsam_points(无 ROS) | 纯单测 |
| `InitialPoseSource` | 订阅 `/initialpose`(2D),持有 config 默认位姿;线程安全 `pop_pending()` 返回 (x,y,yaw) 平面猜测。6DoF 补全(z/roll/pitch/精配准)由 `PriorMapLocalization` 在应用时结合 `X(t)` 与地图完成(见 §2.4) | rclcpp | 逻辑单测 |
| `PriorMapLocalization` | `ExtensionModule`;`on_new_frame` 建 S2M 因子(`X(frame->id)`,源点经 `random_sampling`)暂存进成员 `NonlinearFactorGraph`,`on_smoother_update` flush(范式同 velocity_suppressor);维护 `T_odom_map`、`state`、冷却;发布 TF/话题 | glim, gtsam_points, rclcpp | 集成测 |

### 3.3 状态(MVP 仅两态)

- `UNINITIALIZED`:等待初始位姿。收到 `/initialpose` 或应用 config 默认位姿后,设定 `T_odom_map` 初值 → `TRACKING`。
- `TRACKING`:正常注入 S2M 因子。
- (`INITIALIZING` / `LOST` / 自动重定位 → v2。)

---

## 4. 数据流(每帧)

```
/initialpose(2D: x,y,yaw)─┐
                          ▼
   InitialPoseSource.pop_pending() ──(首帧 或 中途重定位)──►
       在时刻 t,用最新缓存的 X(t) 构造 6DoF T_map_imu(t):       // §2.4
         z          ← PriorMapManager.query_ground_height(x,y) + imu_height_above_ground
         roll,pitch ← X(t) 的重力对齐姿态
         (可选)     ← 一次性 VGICP 对地图精配准
       T_map_odom = T_map_imu(t) · X(t)⁻¹                        // 不假设 X(t)=identity
       若 之前已是 TRACKING(中途重定位):                          // 首帧初始化跳过冷却
         cooldown_until = t + effective_cooldown                 // §2.4:max(配置, smoother_lag+余量)
       state = TRACKING

GLIM odometry ──on_new_frame(frame)──►   // 范式同 velocity_suppressor:此处建因子并暂存
    若 state==TRACKING 且 frame->stamp ≥ cooldown_until:        // 冷却期跳过
      T_map_sensor = T_map_odom · frame->T_world_imu
      overlap = PriorMapManager.compute_overlap_ratio(frame->frame, T_map_sensor)
      // 滞回(s2m_active 为成员状态位,§5.1):避免边界抖动
      若 s2m_active:  s2m_active = (overlap ≥ min_overlap × 0.8)   // 退出阈值
      否则:          s2m_active = (overlap ≥ min_overlap)         // 进入阈值
      若 s2m_active:                                            // 二值注入决策(带滞回)
        src = gtsam_points::random_sampling(frame->frame, s2m_points_ratio, mt)   // 随机抽,非截断
        f   = IntegratedVGICPFactor(T_odom_map, X(frame->id), prior_voxelmap, src)// 固定目标,无噪声模型参数
        f.set_num_threads(s2m_num_threads)
        pending_factors.add(f)                                  // 暂存成员 NonlinearFactorGraph
    // 冷却期 / overlap<min_overlap → 不暂存 → 该帧纯 LIO

GLIM smoother ──on_smoother_update(smoother, new_factors, new_values, new_stamps)──►
    new_factors.add(pending_factors); pending_factors.resize(0)  // flush(velocity_suppressor 同款)
    // X(frame->id) 由 GLIM 在本轮 new_values 中创建,与暂存因子同周期入图 → 满足窗口内约束

(优化由 GLIM 完成;做法 A 下 T_map_odom 为常量,无需反算)
    ▼
发布:map→odom TF(常量 T_map_odom)
      /localization/pose  (PoseStamped, = T_map_odom · T_odom_base)
      /localization/overlap (Float64)
      /localization/status  (String: UNINITIALIZED/TRACKING)
```

**窗口约束**:S2M 因子在 `on_new_frame` 中仅以**当前新帧**的位姿 key `X(frame->id)` 构建,该变量由 GLIM 在紧随的 `on_smoother_update` 同周期 `new_values` 中创建并入图;因子绝不引用历史/已边缘化变量,满足 GLIM「附加因子只能引用优化窗口内变量」的硬约束。范式与 `velocity_suppressor`(对 `V(frame->id)` 注入)完全一致,已对源码核实。

---

## 5. 关键算法

### 5.1 注入决策(MVP 二值;连续降权见 §2.3 留 v2)

带滞回的有状态决策(`s2m_active` 为成员状态位,跨帧保持):

```
update_inject(overlap):                  # 每帧调用,返回是否注入
    if s2m_active:  s2m_active = overlap >= min_overlap * 0.8   # 已激活 → 用退出阈值
    else:           s2m_active = overlap >= min_overlap         # 未激活 → 用进入阈值
    return s2m_active
```

因子无噪声模型(§2.1/§2.3),MVP **不做权重缩放**,注入即用因子原生代价。滞回(进入需 ≥ `min_overlap`,退出需 < `min_overlap × 0.8`)避免地图边界处反复注入/停注的抖动。`full_weight_overlap` 等连续权重参数保留在 config 中但 MVP 不使用,供 v2 wrapper 启用。

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
    "assume_gravity_aligned": true,// 声明地图 z-up 重力对齐(§2.4 前提);MVP 仅告警级地面存在性检查,不拒启
    "voxel_resolution": 0.5,
    // --- S2M 注入与强度(§2.3) ---
    "min_overlap": 0.05,           // 注入 S2M 因子的重叠率阈值(二值决策)
    "max_corr_dist": 1.0,          // m,重叠率命中阈值
    "s2m_points_ratio": 1.0,       // source 随机子采样比例 = S2M 全局强度旋钮
    "s2m_num_threads": 4,          // S2M 因子 set_num_threads()
    "relocalization_cooldown_sec": 5.0, // 中途重定位后冷却(§2.4);启动时下限抬到 max(此值, smoother_lag+余量)
    // --- 初始位姿(§2.4) ---
    "default_init_pose": [0,0,0,0],// x y z yaw(map 系);留空则等待 /initialpose
    "ground_grid_resolution": 1.0, // m,2D 高度格网分辨率
    "imu_height_above_ground": 0.0,// m,IMU 相对地面标称高度
    "refine_initial_pose": true,   // 应用初值后做一次性 VGICP 精配准
    // --- v2 预留(MVP 不读取) ---
    "full_weight_overlap": 0.30,   // 连续降权用
    "use_gpu": false               // S2M 因子 GPU 加速,MVP 固定 false
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

### 6.3 运行启用(⚠️ 现场必读)

1. **注册扩展**:把 `"libprior_map_localization.so"` 加入活动 `config_ros.json` 的 `extension_modules`,否则 `.so` 不会被加载。
2. **避免 `map→odom` TF 冲突**:`rviz_viewer`(及其他)也会广播 `map→odom`(用 GLIM 内部 SLAM 位姿)。与本模块同时运行会在同一 TF 边上 10Hz 互相覆盖、结果不确定。**启用本模块时必须把 `config_ros.json` 的 `map_frame_id` 设为 `""`** 以抑制 GLIM 自带的 `map→odom` 广播。
3. **帧名一致**:本模块 config 的 `map_frame_id`/`odom_frame_id` 默认 `map`/`odom`,应与 `config_ros.json` 的 `glim_ros.*_frame_id` 保持一致。
4. 设置 `prior_map_path` 指向 PCD;首帧用 RViz `2D Pose Estimate`(`/initialpose`)给初值,或在 config 设 `has_default_init_pose: true` + `default_init_pose`。

---

## 7. 测试策略

### 7.1 单元测试(脱 ROS)

| 测试项 | 内容 | 通过标准 |
|--------|------|----------|
| 地图加载 | 加载已知 PCD | 点数与文件一致 |
| 地面高度查询 | 已知斜坡地图,查多个 (x,y) | `query_ground_height` 误差 < 体素分辨率;无点处返回失败 |
| 重叠率 | 构造全重叠 / 零重叠场景 | 误差 < 5% |
| VGICP 因子 | 已知位姿偏移下,因子 `error()` 单调、`linearize()` 收敛到真值 | 配准后位姿误差 < 体素分辨率;复用 gtsam_points 自身因子无需测雅可比 |
| 注入决策 | overlap 跨 min_overlap(含滞回) | 进入/退出阈值与滞回行为正确 |

### 7.2 集成测试

| 场景 | 内容 | 通过标准 |
|------|------|----------|
| 初始定位(平地) | 发 `/initialpose` 后 | `map→odom` 收敛,轨迹贴合地图 |
| 初始定位(坡度) | 在有坡度处发 2D `/initialpose` | z 经地面查询补全后 VGICP 收敛,不发散 |
| 中途重定位 | 运行中(`X(t)≠identity`)再发 `/initialpose` | 用 `X(t)` 正确解算新 `T_map_odom`;**冷却期内无 S2M 注入、跳变后无震荡**,旧因子边缘化出窗后平滑重贴合 |
| 地图内全程 | 全程在地图范围 | APE < 0.1 m |
| 地图缺失区域 | 走出地图 → 返回 | 重叠率→0、因子停注、不漂飞;回到地图后重新收敛(论文核心卖点) |

### 7.3 性能

| 指标 | 目标 |
|------|------|
| 单帧 S2M 注入+优化增量 | < 100 ms(CPU VGICP 因子,中小地图) |
| 先验地图体素图内存 | < 2 GB(典型地图) |

---

## 8. 风险与对策

| 风险 | 对策 |
|------|------|
| 中途重定位后旧 S2M 因子持旧 `fixed_target_pose`,与新对齐自相矛盾(回调无删因子接口) | §2.4 步骤6:重定位后冷却 `effective_cooldown = max(配置, smoother_lag+余量)` 不注入 S2M,纯 LIO 兜底;**首帧初始化不冷却**;集成测验证跳变后无震荡 |
| 冷却时长 < smoother_lag → 冷却结束时窗口仍有旧因子,矛盾约束未消(正确性破坏) | §2.4:启动读 `smoother_lag`,`effective_cooldown` 下限抬到 `smoother_lag+余量`,偏小则自动抬高+告警 |
| S2M 相对 S2S+IMU 强度需调 | §2.3:`s2m_points_ratio` 随机子采样线性缩放(零代码旋钮);严格信息矩阵缩放的 wrapper 留 v2 |
| 后处理 PCD 非 z-up 重力对齐 → §2.4 的 z 查询/roll-pitch 失效 | §6.1 `assume_gravity_aligned` 显式声明担责;MVP 仅**告警级地面存在性检查**(不用主平面法向,避免巷道墙面误判);严格 RANSAC 校验留 v2 |
| 大地图保留原始点云致内存翻倍 | §2.4:加载时预计算 2D 高度格网后释放原始点云,内存 GB→MB |
| GPU VGICP 因子注入 GPU smoother 是否被正确接管 未验证 | MVP 统一用 CPU S2M 因子绕开;use_gpu 加速留 v2 |
| 做法 A 下 `odom→base` 非纯漂移,下游导航可能不适应 | MVP 暂不处理;必要时 v2 引入做法 B |
| 大 PCD 一次性加载显存不足 | MVP 限定中小地图;分块加载留 v2 |
| 2D `/initialpose` 缺 z/roll/pitch,坡度下踢出 VGICP 收敛域 | §2.4:z 从地图地面查询补全、roll/pitch 取自 `X(t)`、一次性 VGICP 精配准 |
| 中途重定位时错误假设 `X(t)=identity` | §2.2:强制从最新帧取 `X(t)`,集成测「中途重定位」覆盖 |
| 初始位姿误差大导致首帧 S2M 误匹配 | MVP 要求 `/initialpose` xy/yaw 足够准 + §2.4 精配准;FPFH 全局初始化留 v2 |
| GLIM 窗口边缘化时序导致因子引用越界 | 仅以当前新帧 `X(frame->id)` 构建因子(范式同 velocity_suppressor,已核实),集成测覆盖 |

---

## 9. 已核实源码事实(实现阶段可直接信用,gtsam_points 1.2.0 / GLIM 本仓库)

| 事实 | 出处 |
|------|------|
| `IntegratedVGICPFactor_ : IntegratedMatchingCostFactor : gtsam::NonlinearFactor`,**非 `NoiseModelFactor1`**;`error()`/`linearize()` 直接 override | `gtsam_points/factors/integrated_vgicp_factor.hpp`, `integrated_matching_cost_factor.hpp` |
| 固定目标 unary ctor:`IntegratedVGICPFactor_(const gtsam::Pose3& fixed_target_pose, gtsam::Key source_key, const GaussianVoxelMap::ConstPtr&, const std::shared_ptr<const SourceFrame>&)`,无 `SharedNoiseModel` 参数 | 同上,第 49 行 |
| 默认模板 `SourceFrame = gtsam_points::PointCloud`;`EstimationFrame::frame` 为 `gtsam_points::PointCloud::ConstPtr` → 直接喂,零转换 | `integrated_vgicp_factor.hpp` 模板头;`glim/odometry/estimation_frame.hpp:96` |
| 因子仅有 `set_num_threads()`、`set_fused_cov_cache_mode()`;另有 `num_inliers()`/`inlier_fraction()`;无 noise/robust/cost-scale | `integrated_vgicp_factor.hpp` |
| GPU 版 `IntegratedVGICPFactorGPU : NonlinearFactorGPU`(基类不同) | `integrated_vgicp_factor_gpu.hpp` |
| `random_sampling(const PointCloud::ConstPtr&, double rate, std::mt19937&) → PointCloudCPU::Ptr` | `gtsam_points/types/point_cloud_cpu.hpp:125` |
| 注入范式:`on_new_frame` 建因子(key `X(frame->id)`,`gtsam::symbol_shorthand::X`)暂存成员 `NonlinearFactorGraph`,`on_smoother_update` 内 `new_factors.add(...); resize(0)` | `glim_ext/modules/odometry/velocity_suppressor/src/glim_ext/velocity_suppressor.cpp` |
| `on_smoother_update(IncrementalFixedLagSmootherExtWithFallback&, NonlinearFactorGraph& new_factors, Values& new_values, std::map<uint64_t,double>& new_stamps)` | `glim/odometry/callbacks.hpp:119` |
| 配置读取:`glim::Config(GlobalConfigExt::get_config_path("config_xxx"))` + `config.param<T>(section, key, default)`;扩展 config 寻址先主 `config.json` 后回退 glim_ext | velocity_suppressor 构造函数;glim_ext README |
| 初始状态估计可设 `NaiveInitialStateEstimation::set_init_state(...)`/`force_init`,但**未向扩展暴露** → 故采用做法 A(§2.2)而非 seed 初始位姿 | `glim/odometry/initial_state_estimation.hpp:63,76` |

---

## 10. 参考

- ROBOMECH2023 译文:`glim-underground/docs/glim/robomech2023_译文.md`
- 原始 proposal(完整版,含 v2 内容):`glim-underground/docs/glim/glim_localization_proposal.md`
- 因子注入参考实现:`glim_ext/modules/odometry/velocity_suppressor/`
- GLIM 回调 API:`glim-underground/include/glim/odometry/callbacks.hpp`
- VGICP 固定目标因子:`gtsam_points/factors/integrated_vgicp_factor.hpp`
