# 消息冻结草案（期 1 入口评审稿）

状态：**待评审**——本文档是冻结提案，不是已生效定义。评审通过后改 msg 并打版本。
对应《GNSS生产线》§3.5"消息冻结"与需求盘点表中的三个 ❌ 项。

## 1. RtkFix 扩展（gnss_msgs/msg/RtkFix.msg）

现有字段（已上线，不动）：header / gnss_time / quality / raw_status /
lat/lon/alt / sigma_enu / diff_age / sats_* / heading 系列 / ratio。

**拟新增**：

```
# --- 速度（ENU，m/s）---
float32 vel_e                 # 东向；0 = 源不提供
float32 vel_n                 # 北向
float32 vel_u                 # 天向
float32[3] vel_std_enu        # 速度标准差；0 = 源不提供

# --- 姿态（deg）---
float32 roll
float32 pitch
float32[3] att_std            # roll/pitch/heading 标准差；0 = 源不提供

# --- INS ---
uint8 ins_status              # 归一化 INS 状态（枚举见下）；0 = INS_INACTIVE
uint8 raw_ins_status          # 源原始值追溯（北云字符串映射表序号 / 华测 sta 低4位）；0 = 不提供

# --- 双天线基线 ---
float32 baseline_length       # m；0 = 源不提供或不适用

# INS 状态归一化枚举（对齐 localization_parser common::InsStatus 值域）
uint8 INS_INACTIVE=0
uint8 INS_ALIGNING=1
uint8 INS_HIGH_VARIANCE=2
uint8 INS_SOLUTION_GOOD=3
uint8 INS_SOLUTION_FREE=4
uint8 INS_ALIGNMENT_COMPLETE=5
uint8 INS_DETERMINING_ORIENTATION=6
uint8 INS_WAITING_INITIALPOS=7
uint8 INS_WAITING_AZIMUTH=8
uint8 INS_INITIALIZING_BIASES=9
uint8 INS_MOTION_DETECT=10
uint8 INS_UNKNOWN=255
```

待定项（评审点）：
- IMU 原始量（陀螺/加表）是否进 RtkFix？方案 §3.5 列了"IMU"但现有消费方
  （flofix/融合）都直接订驱动的 IMU 话题，RtkFix 带 IMU 会造成两份真相。
  **建议：本冻结不纳入**，RtkFix 只带姿态/速度。
- `raw_status` 已有（解质量追溯），`raw_ins_status` 是否合并为一个
  raw 字段包？**建议分开**，两者来源报文不同（北云 pos_type vs ins_status）。

## 2. LocalizationData 扩展（robominer_msgs/msg/LocalizationData.msg）

**拟新增 4 字段**（方案：正式替代 roll/pitch 覆写）：

```
uint8 invalid_reason     # 无效原因分类（六类，见下）；0=有效/无
bool  zero_velocity      # 零速标志（替代 roll 覆写语义）
float32 static_std       # 静态标准差 m（替代 pitch 覆写语义；统计窗内有效）
uint8 judge_source       # 判决来源：0=驱动内一次判断 1=judge 节点二次判决
```

`invalid_reason` 六类（方案 §3.4"严格六类"）：
```
0 = NONE（有效）
1 = 设备异常（硬件告警/基线异常）
2 = 初始化（INS 未收敛/未激活）
3 = 差分龄期大
4 = 卫星信号差
5 = 解算结果异常（横向速度/姿态跳变率，期 2 接入）
6 = 其他（解析失败/超时/无数据/ENU 越界）
```

注意：invalid_reason 与 fault_code 并存而非替代——fault_code 是 9 条规则的
逐帧触发码（数据面），invalid_reason 是面向统计运维的粗分类。两者映射关系
在 gnss_core 内唯一实现。

## 3. 新建 LocalizationMetrics.msg（robominer_msgs）

慢变指标独立话题 `/localization/metrics`（1Hz 或变更时发布）：

```
std_msgs/Header header
# RTK 固定率（时长加权，segment 累计，落盘续跑）
float32 fix_rate_session       # 本次运行段
float32 fix_rate_day           # 当日（UTC+8 日界）
float64 segment_start          # 段起始 unix 秒
float64 valid_duration_s       # 计入统计的总时长（露天+自动驾驶过滤后）
float64 fixed_duration_s       # 固定解时长
# 静态标准差（Welford 在线估计 + 最大偏移 + CEP95）
bool    static_active          # 当前处于静止段（VCU/IMU 判静止——依赖遗留项决策）
float32 static_std_m           # 当前静止段水平 σ
float32 static_max_offset_m    # 当前静止段最大偏移
float32 static_cep95_m         # 当前静止段 CEP95
```

## 4. 冻结纪律（评审确认项）

1. 三个仓库（finder_ros/gnss_msgs、boonray_ws/gnss_msgs、common/robominer_msgs）
   的 msg 改动**必须同 PR 或同周合并**，CI 加消息哈希校验（替代纯纪律）。
2. 字段冻结后，新增字段只能追加、不改语义不改序号；废弃字段保留占位注释。
3. 每个 msg 文件头加 `frozen_at: <日期> / frozen_by: <评审链接>` 注释。
```
