# 动态对比测试方案：生产在跑、新链并行（nvidia@80）

日期：2026-11-18
约束：80 车正在有人全流程自动作业，**不停 robotruck 生产栈、不改 INS 配置、
不占 2222/3333/44445 端口、不动生产话题**。

## 结论

可行，但新链没有现成输入源（生产栈是 parser 直连 INS TCP，总线上没有 /bynav/* 话题），
需要一条旁路数据通路。推荐**方案 A（准实时转发）**，**方案 B（离线回放）**兜底。

关键资产：生产 localization_parser 持续落盘原始 ASCII 流
`~/data/localization_raw/beiyun/ascii/beiyun_ascii_YYYYMMDD_HHMMSS.log`
（按小时切分，实测 ~72KB/s 在写）。流内含 INSPVAXA/CORRIMUDATAA/RAWIMUA/GPGGA/HEADINGA，
正是新链 bynav_adapter 需要的输入。

## 方案 A（推荐）：准实时转发，真·同步并行

```
INS ──44445──> 生产 parser ──> /localization/localization_data（旧链，生产）
                  │（落盘 ascii 文件）
                  ▼ tail -f 增量转发（新增小工具 forward_raw_tail.py）
            本地 TCP 12222 ──> bynav oem7 节点（驱动栈 release 里的 bynav_ros_driver，
                               单独起，receiver_init_commands 置空，不碰 INS）
                          ──> /bynav/inspvax 等话题
                          ──> bynav_adapter → localization_node → /localization_new/*
            ros2 bag 同时录新旧两路输出 + /odom
```

- 延迟：文件落盘缓冲 + tail，预计秒级内；对比分析按时间戳对齐，延迟不影响结论
- 新旧输出话题完全隔离：新链继续用 `/localization_new/*` 重映射
  （**铁律：新链绝不能用默认名 /localization/localization_data**——新旧
  robominer_msgs 定义不同，同名会让生产下游看到类型冲突）
- 全部节点设 `FASTRTPS_DEFAULT_PROFILES_FILE=/usr/config/ros2_dds_config_shm_only.xml`
  （与生产同 profile，确保能看到生产话题）

### 步骤

1. 预检（只读）：生产 `/localization/localization_data` 在发；raw 文件在增长；
   磁盘余量（raw ~260MB/h + bag，需 ≥ 20GB）
2. 部署转发链（作业间隙或作业中均可，全是只读+旁路）：
   - oem7 节点单独起（自定义 params：oem_port=12222，init_commands 空，话题名加 _new 前缀）
   - forward_raw_tail.py：跟踪最新 ascii 文件（处理整点轮转），推 12222
   - 新链 ws_new 已有（80 车上昨晚的还在），origin 改回矿区原点（车在矿区作业！）
     ——用 `localization_params_file` 单独给一份矿区原点 yaml
3. 起新链 + 双 bag 录制（新链输出 + 生产 /localization/localization_data 及 aux + /odom）
4. 跑一个作业班次（≥2h，含行驶/转弯/装卸），期间每小时巡检
5. 收工后离线分析，出报告

## 方案 B（兜底）：离线回放

作业结束后拿当天的 ascii raw 文件，在 80（或开发机）上回放：
回放器按原速/加速推 TCP → oem7 节点 → 新链，与生产同时段输出对比。
零生产侵入，缺点是非实时、时间对齐要小心（用 GPS 周内秒对齐）。

## 安全红线

| 红线 | 措施 |
|---|---|
| 不停/不动生产 robotruck | 全部旁路部署，无 kill、无重启 |
| 不改 INS 配置 | oem7 节点 init_commands 置空；不发任何 ICOM 命令 |
| 不占生产端口 | 新链侧只用新端口 12222 |
| 不动生产话题 | 新链输出全在 /localization_new/*；fault/heartbeat 同理 |
| 生产下游类型安全 | 同上（消息版本不同，同名即冲突） |

## 待确认（需用户决策）

1. **生产定位当前 is_valid=false（solnsvs=0，GPFPD 缺失）**——作业是否正常？
   这决定要不要先修生产（可能是 slotB 老版本 parser 用 GGA 判星数、无碍；
   也可能生产真的 degraded）。请现场/调度确认。
2. 新链 origin 用矿区原点（与生产一致）——具体值从生产配置抄，部署时确认。
3. 转发方案 A 需要作业中在场边远程操作 20 分钟，是否允许？

## 风险

- tail 转发若遇整点文件轮转有秒级断流 → 对比分析按时间段剔除
- 作业中网络/CPU 余量：80 是 Orin，昨晚构建+录制 CPU 占用不高，生产不受影响
  （转发链全程 < 5% CPU）
