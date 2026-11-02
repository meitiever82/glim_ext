# rtk-monitor 部署指南

## 1. 硬件与网络前置

### 1.1 硬件要求

- **主机**：ARM64 架构（NVIDIA Orin、RK3588 等）
- **网络**：车载以太网（有线）
- **CAN 总线**：SocketCAN 接口 `can0`（需驱动支持）
- **存储**：最少 50 GB 用于滚动日志（按 `retention_days` 清理）

### 1.2 网络连通性核对

1. **基站/平台广播端口**（差分 RTCM）：
   - 配置项：`corrections.host`（默认 `192.168.10.1`）、`corrections.port`（默认 `6001`）
   - 验证命令：`nc -zv 192.168.10.1 6001`

2. **610 板卡数据（Server7、Client7）**：
   - Server7 输出（原始观测）：`raw_obs.host` 配置的 TCP 服务器（默认 `192.168.200.1:9901`）
   - Client7 输出（GPCHC 解算）：`gnss_solution.host` 监听地址（默认 `0.0.0.0:9902`，610 需回连此地址）
   - 验证方法：在 610 网页配置界面中测试端口连通性

3. **CAN 总线**：
   - `can0` 接口可正常收发
   - 验证命令：`candump can0`（应能看到周期性的 CAN 帧）

---

## 2. 610 网页配置步骤

### 2.1 打开 610 管理界面

1. 从车载电脑浏览器打开 610 网页地址（IP 由现场管理员确认，通常 `192.168.200.1`）
2. 进入 **I/O 设置** 或 **Data Output** 菜单

### 2.2 Server7（原始板卡观测）配置

1. 找到 **Server7** 或 **TCP 服务** 相关项
2. **勾选** `GNSS 板卡数据`（不勾其他项）
3. 端口设置为 `9901`（与 `raw_obs.port` 一致）
4. 保存配置

**说明**：原始观测数据将被 app 独立解算为 RTK，作为诊断参考。

### 2.3 Client7（纯卫导解）配置

1. 找到 **Client7** 或 **TCP 客户端** 相关项
2. **勾选** `卫导数据`（选择 GPCHC 或 HCINSPVATZCB 格式）
3. 勾选 `IMU 原始数据`（可选，用于检查 IMU 链路健康）
4. **勾选** `CAN 输出`（同时打开 CAN 50Hz 融合结果输出）
5. **连接方式**确认：
   - 如果 610 配置为"连接远端服务器"，填写车载电脑 IP 和 `gnss_solution.port`（默认 `9902`）
   - 如果配置为"被动监听"，确保监听地址配置为 `0.0.0.0:9902`
6. 保存配置

**说明**：卫导解与 CAN 融合结果是诊断的关键对比源。

### 2.4 差分接入方式

1. 找到 **差分 RTK 设置** 或 **Base RTK** 菜单
2. 确认接入方式为 **TCP 客户端**
3. 连接地址为基站/平台广播端口（`corrections.host:corrections.port`）
4. **不关闭** CAN 输出

参考规范详情见 [设计文档 §1.2](superpowers/specs/2026-08-31-rtk-monitor-design.md#12-外部环境现场事实)。

---

## 3. 配置文件说明

### 3.1 配置文件位置与拷贝

1. 复制示例配置：
   ```bash
   cp config.yaml.example config.yaml
   ```

2. 编辑 `config.yaml` 并根据现场环境调整（见下表）

3. 容器启动时，将 `config.yaml` 挂载到 `/data/config.yaml`

### 3.2 完整字段说明表

| 配置项 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| **数据根路径** | | | |
| `data_root` | 字符串 | `/data/gnsslog` | 落盘数据的根目录 |
| `db_path` | 字符串 | `/data/gnsslog/rtk_monitor.db` | SQLite 数据库路径 |
| **差分链路（Route 1）** | | | |
| `corrections.host` | 字符串 | `192.168.10.1` | 基站/平台广播服务器 IP |
| `corrections.port` | 整数 | `6001` | 基站/平台广播 RTCM 端口 |
| **原始观测（Route 2）** | | | |
| `raw_obs.host` | 字符串 | `192.168.200.1` | 610 Server7 地址 |
| `raw_obs.port` | 整数 | `9901` | 610 Server7 端口 |
| **纯卫导解（Route 3）** | | | |
| `gnss_solution.host` | 字符串 | `0.0.0.0` | 本地监听地址（接收 610 Client7 回传） |
| `gnss_solution.port` | 整数 | `9902` | 监听端口 |
| `gnss_solution.listen` | 布尔 | `true` | `true` = 被动监听，`false` = 主动连接 |
| **CAN 总线** | | | |
| `can_channel` | 字符串 | `can0` | SocketCAN 接口名 |
| **储备转发端口（供 rtkrcv）** | | | |
| `reserve.corrections_port` | 整数 | `15010` | 本地转发差分数据端口 |
| `reserve.raw_obs_port` | 整数 | `15011` | 本地转发原始观测端口 |
| **数据保留与清理** | | | |
| `retention_days` | 整数 | `14` | 裸流文件保留天数 |
| `disk_watermark_pct` | 浮点 | `85.0` | 磁盘使用率超过此值（%）时触发清理 |
| `db_retention_days` | 整数 | `30` | SQLite 历元/事件记录保留天数 |
| **rtkrcv 独立解算（可选）** | | | |
| `rtkrcv.binary` | 字符串 | `""` | rtkrcv 二进制路径；空字符串表示禁用 |
| `rtkrcv.sol_port` | 整数 | `15020` | rtkrcv 输出解算结果的监听端口 |
| **诊断阈值（缺省键使用设计规范默认值）** | | | |
| `diagnosis.corr_gap_s` | 浮点 | `3.0` | 差分中断告警阈值（秒） |
| `diagnosis.age_max_s` | 浮点 | `10.0` | 差分龄期告警阈值（秒） |
| `diagnosis.base_shift_m` | 浮点 | `0.1` | 基站坐标偏移告警阈值（米） |
| `diagnosis.min_sats` | 整数 | `6` | 卫星不足告警阈值（颗数）|
| `diagnosis.resid_max_m` | 浮点 | `2.0` | 伪距残差告警阈值（米） |
| `diagnosis.low_el_deg` | 浮点 | `20.0` | 低高度角阈值（度） |
| `diagnosis.low_snr_dbhz` | 浮点 | `35.0` | 低信噪比阈值（dBHz） |
| `diagnosis.min_ratio` | 浮点 | `3.0` | 模糊度固定 ratio 阈值 |
| `diagnosis.slip_max_per_30s` | 整数 | `5` | 周跳频发告警阈值（次/30秒） |
| `diagnosis.divergence_sigma` | 浮点 | `3.0` | 610 与独立解偏差告警倍数（σ） |
| `diagnosis.divergence_hold_s` | 浮点 | `5.0` | 610 与独立解偏差告警持续时长（秒） |
| `diagnosis.close_hysteresis_s` | 浮点 | `10.0` | 事件关闭延迟（迟滞，秒） |
| `diagnosis.sol_stale_s` | 浮点 | `5.0` | 解算新鲜度判定阈值（秒） |
| **其他发布（可选）** | | | |
| `publish.enabled` | 布尔 | `false` | 是否启用 UDP JSON Lines 发布 |
| `publish.host` | 字符串 | `127.0.0.1` | UDP 发布目标 IP |
| `publish.port` | 整数 | `15030` | UDP 发布端口 |
| **Web 界面** | | | |
| `web.port` | 整数 | `8080` | FastAPI Web 服务端口 |
| `web.host` | 字符串 | `0.0.0.0` | Web 服务绑定地址 |
| `web.static_dir` | 字符串 | `""` | 静态文件目录；空字符串表示使用仓库相对路径 `web/` |
| `web.tiles_path` | 字符串 | `""` | MBTiles 卫星影像文件路径；空字符串表示界面自动降级为网格 |

**注意**：诊断阈值可根据现场情况调整，但建议先在默认值下运行 48 小时观察效果。

---

## 4. 裸机运行

### 4.1 环境准备

1. 确保已在目标机构建 rtkrcv：
   ```bash
   bash scripts/build_rtkrcv.sh
   ```

2. 验证 Python 3.11+ 环境：
   ```bash
   python3 --version
   ```

3. 安装依赖：
   ```bash
   python3 -m pip install .
   ```

### 4.2 启动应用

1. 复制配置文件：
   ```bash
   cp config.yaml.example config.yaml
   ```

2. 编辑配置（参考 [§3.2](#32-完整字段说明表)）

3. 启动应用：
   ```bash
   python3 -m rtk_monitor.main
   ```

4. 打开浏览器访问：`http://localhost:8080`（或车载电脑 IP：`http://<car_ip>:8080`）

### 4.3 查看日志

应用未落盘日志文件，日志写到进程 stderr（`main()` 已配置
`logging.basicConfig(level=logging.INFO, ...)`）。前台运行时直接可见；
后台/托管运行时按需重定向或交给进程管理器（systemd/supervisor 等）采集：

```bash
# 例：systemd 托管时查看日志
journalctl -u rtk-monitor -f

# 查看进程与 rtkrcv 子进程
ps aux | grep rtkrcv
```

---

## 5. Docker 容器运行

### 5.1 构建镜像

镜像 Dockerfile 随交付包附送（Task 9）。构建命令：

```bash
docker build -t rtk-monitor:latest .
```

**重要**：Dockerfile 构建阶段需要从 GitHub 克隆并编译 RTKLIB（demo5，已固定到具体 commit，
见 Dockerfile 中 `RTKLIB_DEMO5_SHA` 注释），镜像须在有网环境预构建（ARM64 主机或使用
`docker buildx` 交叉构建），矿区现场网络通常无法访问外网，无法在现场重新构建镜像——现场
只需 `docker load`/`docker compose up -d` 已构建好的镜像。

### 5.2 启动容器

使用 Docker Compose（`docker-compose.yml`）：

```yaml
version: '3.8'
services:
  rtk-monitor:
    build: .
    restart: always
    network_mode: host
    volumes:
      - /data:/data
```

运行：
```bash
docker compose up -d --build
```

**关键配置**：
- `build: .` — 从 Dockerfile 构建镜像
- `network_mode: host` — 共享主机网络栈（必须，用于访问 SocketCAN 和 TCP 端口）
- `/data:/data` — 挂载数据卷，存储日志、数据库、裸流
- `restart: always` — 容器异常退出时自动重启
- **注意**：SocketCAN 是网络命名空间接口，不需要 `/dev` 字符设备映射；`network_mode: host` 已覆盖
- **重要**：`/data/config.yaml` 必须显式设置 `web.static_dir: /app/web`。容器内通过
  `pip install .` 安装包后，源码不再与仓库 `web/` 目录相邻，`web.static_dir` 的默认空字符串
  （仓库相对路径回退）在容器中无法解析静态文件目录，需显式指向 Dockerfile 中 `COPY web ./web`
  落盘的 `/app/web`。

### 5.3 验证容器运行

```bash
# 查看容器日志
docker logs -f rtk-monitor

# 验证 Web 服务
curl http://localhost:8080
```

### 5.4 停止与清理

```bash
# 停止并移除容器
docker compose down
```

`docker-compose.yml` 用的是 bind mount（`/data:/data`），不是具名 volume，
`docker compose down -v` 的 `-v` 只清理具名 volume，对 `/data` 下的数据没有影响——
两条命令效果相同，日常使用 `docker compose down` 即可。若确实要清空 `/data`
下的数据（config.yaml、gnsslog、rtk.db、mbtiles），需手动 `rm -rf /data/...`，请谨慎操作。

---

## 6. 安全姿态

### 6.1 当前假设

- **部署网络**：可信局域网（车载以太网、无外部路由）
- **鉴权**：无（所有 WebSocket、API、报告接口均无密钥验证）
- **访问控制**：依赖网络物理隔离

### 6.2 已知风险与加固方向

1. **Web 界面无鉴权**：
   - 当前：仅在可信局域网内部署，风险可控
   - 加固方向：如需跨域或公网暴露，应在反向代理（nginx）层添加基本认证或 API 密钥

2. **`POST /api/base_reset`**（基站坐标重置）：
   - 当前：无操作验证、无审计日志
   - 加固方向：添加操作确认、记录修改者与时间戳

3. **WebSocket 连接限制**：
   - 当前：对连接数/消息速率无限制
   - 加固方向：在局域网多租户或公网场景下，添加速率限制

参考详细安全分析见 [WebSocket 消息契约](ws-contract.md#7-安全姿态现状说明非本次变更范围)。

---

## 7. 常见问题排查

### 7.1 rtkrcv 进程频繁重启或不启动

**表现**：
- 事件表中频繁出现 `rtkrcv connected/disconnected` 切换
- Web 界面状态条无 RTK 固定解

**排查步骤**：
1. 检查 `rtkrcv.binary` 配置项是否指向有效的二进制文件：
   ```bash
   file $(grep rtkrcv.binary config.yaml | cut -d: -f2 | tr -d ' ')
   ```
   
2. 验证二进制在目标平台可执行：
   ```bash
   <binary_path> -h
   ```
   
3. 查看日志中的 rtkrcv 启动错误（容器运行用 `docker logs -f rtk-monitor`；
   裸机运行看进程 stderr，见 [§4.3](#43-查看日志)）：
   ```bash
   docker logs rtk-monitor 2>&1 | grep -i "rtkrcv\|solver" | tail -20
   ```

4. 如果 rtkrcv 无法编译或运行，参考 [rtkrcv 真机集成核对清单](integration-rtkrcv.md#rtkrcv-真机集成核对清单) 逐项核对

### 7.2 Web 界面打开后无数据

**表现**：
- 浏览器能访问 `http://localhost:8080`，但状态条为灰色、轨迹/事件为空

**排查步骤**：
1. 验证 WebSocket 连接（浏览器开发者工具 → Network → WS）：
   - 应有 `ws://localhost:8080/ws` 连接处于 101 Switching Protocols
   
2. 检查采集链路是否有数据进入：
   ```bash
   # 查看事件表（应有 corrections_link / raw_obs_link / gnss_solution_link / can_collector 等）
   sqlite3 rtk_monitor.db "SELECT * FROM events WHERE etype IN ('corrections_link', 'raw_obs_link') ORDER BY t DESC LIMIT 10;"
   ```
   
3. 如果采集链路连接正常但无数据，检查 610 和基站/平台配置（见 [§2](#2-610-网页配置步骤)）：
   ```bash
   # 尝试手动连接 610 Server7，验证原始观测是否下发
   nc -v 192.168.200.1 9901
   ```
   
4. 检查 CAN 总线是否有帧到达：
   ```bash
   timeout 5 candump can0 | head
   ```

### 7.3 基站坐标突然变动、全矿车辆位置整体平移

**原因**：基站坐标在 RTCM 1005/1006 电文中发生变化（可能为硬件移位、坐标系校准等）

**诊断**：
- Web 界面状态条会显示黄色告警「基站坐标变动 Xm」
- 事件表中出现相应的诊断事件

**处理**：
1. 现场确认基站是否实际移位
2. 如确认为真实变动，通过 API 更新基准（Web 界面当前无对应按钮，仅提供 API 入口；
   取用最近一次观测到的 1005 电文坐标作为新基准）：
   ```bash
   curl -X POST http://<车IP>:8080/api/base_reset
   ```
3. 后续偏移会以新坐标为基准

### 7.4 通过 kill 进程模拟 rtkrcv 故障

**演练**：
1. 记录当前时刻（用于后续查阅回放）
2. 杀死 rtkrcv 进程：
   ```bash
   pkill -f "rtkrcv -"
   ```
3. **预期行为**：
   - Web 界面状态条**文字**在约 6 秒内变为"独立解算无输出——rtkrcv 未运行或未收敛"，但**徽章颜色仍按 CAN 融合解显示**（通常仍为绿色）；仅当 CAN 也断开时才变为灰色"无数据"
   - 事件表出现诊断事件
   - 应用自动重启 rtkrcv 子进程（见进程监控日志）
4. 验证恢复：观察状态条是否在 2-5 分钟内恢复为绿色（根据数据量和 RTK 锁定速度变化）

---

## 附录：网络拓扑示例

```
┌──────────────┐         ┌──────────────────┐         ┌──────────────────┐
│   基站/平台  │ RTCM    │  车载以太网交换机│TCP      │  车载电脑(ARM64) │
│  (外部)      │ 5G→TCP  │                  │9901/902 │   ┌────────────┐ │
└──────────────┘         └──────────────────┘         │   │ rtk-monitor│ │
                                  ▲                    │   │ + rtkrcv   │ │
                                  │ GPCHC             │   └────────────┘ │
                          ┌───────┴─────────┐         │   ┌────────────┐ │
                          │   610 组合导航  │ CAN     │   │ SocketCAN  │ │
                          │    (网页 I/O)   │◄────────┼───┤   can0     │ │
                          └─────────────────┘         │   └────────────┘ │
                                                      │   浏览器 :8080    │
                                                      └──────────────────┘
```

---

## 相关文档

- [WebSocket 消息契约](ws-contract.md) — 实时/回放数据格式、安全假设
- [MBTiles 卫星影像说明](tiles-howto.md) — 离线地图瓦片制作与集成
- [rtkrcv 真机集成核对清单](integration-rtkrcv.md) — 上车测试与故障排查
- [设计规范](superpowers/specs/2026-08-31-rtk-monitor-design.md) — 架构、诊断规则链、外部环境事实
