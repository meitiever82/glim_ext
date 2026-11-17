# gnss_bringup 车端部署 SOP

版本：v2.0（2026-11-17）
适用：ROS2 Humble + Ubuntu 22.04，北云 OEM7 或华测 610 组合导航
变更：v2.0 将定位链拆分为 adapter + localization_node 架构：
  - 北云 OEM7 用 `bynav_adapter` 输出统一 `gnss_msgs/RtkFix`
  - 华测 610 用 `huace_adapter` 输出统一 `gnss_msgs/RtkFix`
  - `localization_node` 统一做门控并输出 `LocalizationData`
  - 不再使用 `localization_bridge` / `rtkfix_judge`

⚠️ 三条铁律：
1. **开发、测试、提交在 `glim_ws` 完成**，验证通过后再同步到 `localization` 生产仓库。
2. **ENU 原点必须和现场 map 原点一致**，换场地必须同步改 `gnss_bringup_localization.yaml`。
3. **`enable_bynav_adapter` 与 `enable_huace_adapter` 不要同时开**，会向同一个 `/gnss/rtkfix` 发布两份数据。

---

## 0. 构建与发布

### 0.1 开发机构建（glim_ws）

```bash
cd /home/boonraygrn/glim_ws
bash src/glim_ext/setup_workspace.sh        # 首次克隆后必须执行
source /opt/ros/humble/setup.bash
colcon build --packages-select gnss_core gnss_bringup gnss_msgs robominer_msgs
source install/setup.bash

# 回归测试
cd build/gnss_bringup
./test_localization_gate
./test_localization_mapping
./test_loc_alarm_state
cd ../gnss_core
./test_judge
```

### 0.2 同步到 localization 生产仓库

```bash
cd /home/boonraygrn/boonray_ws/src/localization
git checkout 803-fix
# 将 glim_ext 当前 gnss_core、gnss_bringup、golden、doc 拷贝/提交到本仓库
git add gnss_core gnss_bringup golden doc
git commit -m "sync gnss code/docs/golden from glim_ext"
git push origin 803-fix
```

---

## A. 上车前准备

### A1 确认车辆硬件型号

| 项目 | 北云 OEM7 | 华测 610 |
|---|---|---|
| adapter | `bynav_adapter` | `huace_adapter` |
| 定位节点 | `localization_node` | `localization_node` |
| 驱动话题 | `/bynav/inspvax` 等 | `/gnss/rtkfix` |
| IMU 源 | `corrimudata` | 已含在 `RtkFix` |
| 启用 adapter | `enable_bynav_adapter:=true` | `enable_huace_adapter:=true` |
| 启用定位节点 | `enable_localization_node:=true` | `enable_localization_node:=true` |

### A2 确认 ENU 原点

从现场北云/华测驱动配置或地图配置中抄取 `origin_lat`、`origin_lon`、`origin_height`，写入：

```
gnss_bringup/config/gnss_bringup_localization.yaml
```

⚠️ 原点错误会导致 `world→map` 静态 TF 失效、地图整体平移。

### A3 确认话题名

北云检查：
- `/bynav/inspvax`
- `/bynav/bestpos`
- `/bynav/corrimudata`
- `/bynav/heading2`

610 检查：
- `/gnss/rtkfix`（或驱动实际发布的话题名）

### A4 确认 IMU 频率（仅北云）

`config/gnss_bringup_localization.yaml` 中 `imu_rate_hz` 默认 100 Hz，必须与整车 CORRIMU 输出频率一致。乘错会整体缩放角速度/加速度。

---

## B. 车端部署

### B1 安装 RTKLIB-EX 2.5.1（如需要独立解）

```bash
# 见 gnss_bringup/README.md §安装 RTKLIB-EX 2.5.1
# 只跑主定位链 + metrics 时可以不装
```

### B2 构建工作区

```bash
cd <vehicle_ws>
bash src/glim_ext/setup_workspace.sh
source /opt/ros/humble/setup.bash
colcon build --packages-select gnss_core gnss_bringup gnss_msgs robominer_msgs
source install/setup.bash
```

### B3 配置参数

复制并修改现场配置：

```bash
cp src/glim_ext/gnss_bringup/config/gnss_bringup_localization.yaml \
   config/gnss_bringup_localization_<site>.yaml
```

必须修改的字段：
- `origin_lat`、`origin_lon`、`origin_height`
- 话题名（如现场不一致）
- `imu_rate_hz`（北云）
- `rtk_offset_yaw`（如有安装偏角）

### B4 启动主定位链

#### 北云 OEM7

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py \
  enable_bridge:=false \
  enable_bynav_adapter:=true \
  enable_localization_node:=true \
  enable_metrics:=true \
  enable_divergence:=false \
  localization_params_file:=<现场 yaml 路径>
```

#### 华测 610

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py \
  enable_bridge:=false \
  enable_huace_adapter:=true \
  enable_localization_node:=true \
  enable_metrics:=true \
  enable_divergence:=false \
  localization_params_file:=<现场 yaml 路径>
```

### B5 带独立解验证（推荐首次上车或怀疑设备解时）

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py \
  enable_bridge:=false \
  enable_bynav_adapter:=true \
  enable_localization_node:=true \
  enable_metrics:=true \
  enable_divergence:=true \
  enable_rtkrcv:=true \
  localization_params_file:=<现场 yaml 路径>
```

> 华测场景将 `enable_bynav_adapter` 换成 `enable_huace_adapter`。

---

## C. 验收检查

### C1 话题存在且有数据

```bash
ros2 topic hz /localization/localization_data
ros2 topic hz /localization/localization_data_aux
ros2 topic hz /localization/metrics
```

- `localization_data` 应约 50 Hz
- `metrics` 应约 1 Hz

### C2 LocalizationData 字段合理

```bash
ros2 topic echo --once /localization/localization_data
```

检查：
- `is_valid` 在 RTK 固定解且 INS 收敛时为 `true`
- `rtk_status` 与设备质量一致（4 = fixed）
- `x`、`y`、`z` 在合理范围
- `speed` 与车辆实际速度一致

### C3 告警不持续触发（首次上车后 10s 启动抑制结束）

```bash
ros2 topic echo /fault_code/localization_fault
```

稳定 RTK fixed 后应无新的 `FAULT_ADD`。

### C4 divergence 在阈值内（如启用）

```bash
ros2 topic echo /diagnostics
```

查看 `gnss: device_divergence` 的 `divergence_m` 与 `threshold_m`，持续超限需排查设备解或 rtkrcv 配置。

### C5 录包验证（至少 2 分钟）

```bash
bash src/glim_ext/gnss_bringup/scripts/record_gnss.sh
```

回放时检查 `localization_data` 是否连续、无跳变。

---

## D. 收尾

- [ ] 将现场 yaml 命名归档：`config/gnss_bringup_localization_<场地>_<日期>.yaml`
- [ ] 记录验收结果到 `docs/deploy-<日期>-<车号>.md`
- [ ] 有代码/配置变更时，回到 `glim_ws` 提交，再同步到 `localization/803-fix`

---

## 还原与清理

```bash
# 停掉 bringup
pkill -f "gnss_bringup.launch.py"

# 如需彻底还原旧 localization_parser 链路
ros2 launch localization_parser localization_parser.launch.py
```

---

## 常见坑速查

| 症状 | 原因 | 处置 |
|---|---|---|
| `/localization/localization_data` 无数据 | 节点未启用或驱动话题名不对 | 检查 `enable_bynav_adapter`/`enable_huace_adapter`/`enable_localization_node`；用 `ros2 topic list` 核对话题名 |
| `is_valid` 一直 false | INS 未收敛 / 卫星不足 / 原点错误 | 查看 `LocalizationDataAux.fault_code`；确认 `origin_lat/lon/height` |
| `x/y/z` 数值巨大 | ENU 原点与驱动 map 原点不一致 | 对齐原点后重启 |
| 速度/角速度明显偏大（北云） | `imu_rate_hz` 与实际不符 | 改为正确 CORRIMU 频率 |
| `metrics` 固定率为 0 | 设备未进入 fixed | 检查天线、差分、环境遮挡 |
| `divergence` 持续告警 | 设备解与 rtkrcv 独立解偏差大 | 检查基站坐标、差分链路、rtkrcv 配置 |
| 同时起 `bynav_adapter` 和 `huace_adapter` | 两 adapter 发布同名 `/gnss/rtkfix` | 只启用其中一个 |
| colcon 找不到 `robominer_msgs` | 未 source boonray_ws overlay | `source <boonray_ws>/install/setup.bash` 后再编 |
