# gnss_bringup

两个 ROS2 节点,构成 GNSS 数据面的第一段:

- **`rtcm_bridge`**——把裸 TCP 字节流(平台差分、板卡原始观测)桥接到 ROS 话题
  (`gnss_msgs/RawStream`)。不做任何解析,支持连接(`listen=false`)或监听
  (`listen=true`)两种方向,断线自动重连/继续等待。
- **`rtkrcv_node`**——生成 `rtkrcv.conf`、监管 RTKLIB `rtkrcv` 子进程、把它的
  llh 解算流转成 `gnss_msgs/RtkFix` 发布、把 `$SAT` 状态行原样转发到 `~/stat`
  供后续诊断消费。

## 快速开始

```bash
source install/setup.bash
ros2 launch gnss_bringup gnss_bringup.launch.py
```

参数集中在 `config/gnss_bringup.yaml`,launch 文件默认加载它;可用
`params_file:=<path>` 覆盖整份文件,或 `-p <name>:=<value>` 覆盖单个参数。

`enable_rtkrcv:=false` 只起 `rtcm_bridge`(RTKLIB 未安装、或只想验证桥接这一段时用,
见下面"不装 RTKLIB 也能跑起来"一节)。

## 部署要求(必读):`rtkrcv_node` 必须跑在 systemd + cgroup `KillMode` 下

**这是本分支目前最大的残留风险,不是"锦上添花"的建议。**

`ProcessSupervisor` 用 `setsid()` 把 `rtkrcv` 放进独立进程组,这是为了让
`stop()` 能对整组发信号收尾派生出来的孙进程——这个决定本身是对的。但它的
代价是:`rtkrcv` 子进程只在父进程走**正常的 `stop()`/析构路径**时才会被
一起收尾;父进程如果是被 `kill -9`(或者任何不给它机会跑完析构函数的方式,
比如 OOM killer)杀掉的,`rtkrcv` 子进程会被 init 收养、继续独立运行,
变成一个孤儿进程,而 `rtkrcv_node` 完全不知道这件事。

已实测复现的后果——而且是那种**看起来一切正常、实则数据完全错误**的最坏
情况:

1. `kill -9 <rtkrcv_node 的 pid>`。
2. 孤儿 `rtkrcv` 继续拿着 `sol_port`(以及 `corr_port`/`obs_port`)当
   TCP 服务端挂着。
3. 重新拉起 `rtkrcv_node`:它自己的 `ProcessSupervisor` 会尝试再起一个
   **新的** `rtkrcv`,而这个新进程会因为端口被孤儿占着而绑定失败、不断
   崩溃重启——但这不是唯一的坏结果。
4. 更隐蔽的是:`rtkrcv_node` 里负责去**连**解算输出端口(`sol_port`)的
   `TcpStream` 是客户端角色,它会连上"当前监听在这个端口上的任何东西"——
   也就是那个孤儿 `rtkrcv`,而不是新起的那个。孤儿继续吐着它自己那份
   (可能早已过时、来自另一次运行的)解算结果,`rtkrcv_node` 原样转发,
   只是套上一个全新的 `header.stamp`——下游拿到的是一条**时间戳新鲜、
   内容却是旧的/错的定位解**,没有任何报错或状态异常可以据此发现问题。

**现在的规避方式(必须遵守,直到有代码层面的修复)**:

- `rtkrcv_node` 必须由 systemd 管理,并且 unit 文件里配置基于 cgroup 的
  `KillMode=control-group`(而不是默认的 `KillMode=process`)——这样
  systemd 停止/重启这个服务时,会对整个 cgroup(包含孤儿 `rtkrcv`)发信号,
  而不只是对 `rtkrcv_node` 自己的 pid。
- 如果确实发生了 `kill -9`(或者进程以任何方式被信号杀死而不是走
  `ros2 lifecycle`/`Ctrl-C` 之类的正常关闭路径),**在重启 `rtkrcv_node`
  之前必须先手动确认没有孤儿 `rtkrcv` 残留**(例如 `pgrep -a rtkrcv`),
  有的话先手动杀掉,再重启节点。跳过这一步、直接重启,现场表现可能是
  "看起来正常运行,却在发布过时/错误的解"这种最难排查的故障。

代码层面的正确修复需要一个设计决策(`prctl(PR_SET_PDEATHSIG)`——注意它是
线程级语义,`ProcessSupervisor` 的 fork() 发生在一个专用线程上,细节需要
仔细核对;或者在 `run_dir` 下维护一个 pidfile,由下一次启动时先检查再决定
要不要清理),刻意不在这一轮修复范围内,留给后续任务处理。

## 现场待确认(spec §13 P0)

以下四项在现场设备到位前只能先用占位值。协议本身已经确认——`rtcm_bridge` 是裸
TCP、不含 NTRIP(参考实现 rtk-monitor 全仓无 NTRIP);待确认的只是端点、方向、
和两个 `inpstr*-format` 的取值。**这四项都只需要改 `config/gnss_bringup.yaml`
里的值,不需要改任何代码。**

| # | 待确认                               | 影响                             | 现在的取值                            | 确认后怎么改                                                                                         |
| - | ------------------------------------ | -------------------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| 1 | 平台差分的 IP:Port,以及连接方向      | `rtcm_bridge` 连不上就没有差分 | `127.0.0.1:15031`, `listen=false` | 只改`config/gnss_bringup.yaml` 里 `rtcm_bridge.rtcm_corrections` 的 `host`/`port`/`listen` |
| 2 | 板卡原始观测的端点                   | `rtkrcv` 无观测无法解算        | `127.0.0.1:15032`, `listen=false` | 只改`config/gnss_bringup.yaml` 里 `rtcm_bridge.raw_obs` 的 `host`/`port`/`listen`          |
| 3 | 板卡原始观测格式(`inpstr1-format`) | 格式错`rtkrcv` 解不出          | `rtcm3`(沿用 rtk-monitor 假定)      | 只改`config/gnss_bringup.yaml` 里 `rtkrcv_node.obs_format`                                       |
| 4 | 平台差分格式(`inpstr2-format`)     | 同上                             | `rtcm3`                             | 只改`config/gnss_bringup.yaml` 里 `rtkrcv_node.corr_format`                                      |

## 参数说明

`config/gnss_bringup.yaml` 里的大部分参数名直接对应 RTKLIB 的 conf 键
(`pos_mode`、`navsys`、`elmask`、`ar_mode` 等),或是显而易见的端点配置
(`host`/`port`/`listen`/`topic`)。ROS 对未声明的 YAML key 是**静默忽略**的——
写错一个参数名不会报错,只是那个设置根本没生效——所以下面两组容易被漏掉的参数
专门说明一下:

- **`leap_seconds`(默认 18)**——GPST 与 UTC 之间的闰秒偏移量。这个值不是常量,
  IERS 每次宣布插入新闰秒后都需要手动更新;`rtkrcv_node` 用它把解算历元
  (GPST)换算成 UTC。
- **退避/超时参数**(`rtkrcv_node` 的 `restart_delay_s`/`crash_loop_life_s`/
  `max_restart_delay_s`(子进程重启退避)、`sol_initial_backoff_s`/
  `sol_max_backoff_s`/`sol_idle_timeout_s`(连接 `rtkrcv` 解算输出口的重连与
  空闲检测),以及 `rtcm_bridge` 每路流各自的 `initial_backoff_s`/
  `max_backoff_s`/`idle_timeout_s`)——这些是隧道内弱链路场景下运维最可能需要
  现场调整的旋钮:链路抖动大就适当放宽退避上限和空闲超时,链路稳定就可以收紧
  以更快发现故障。

  **`sol_idle_timeout_s` 不接受 0**:节点会在启动时拒绝这个值。0 会被底层
  `TcpStream` 当成"彻底关闭空闲检测"的哨兵值,而这条自愈能力
  ("`rtkrcv` 连接卡死了要能被自动发现并恢复")正是这个节点存在的核心意义,
  不允许通过配置在现场被悄悄关掉。

完整参数表见 `config/gnss_bringup.yaml` 里的注释——文件本身就是文档,每个参数
旁边都带着来源(哪个节点声明、默认值多少)。

## 未验证项

本包开发机上**没有安装 RTKLIB**,以下事项只用 `test/fake_rtkrcv.sh`(一个模拟
`rtkrcv` 命令行行为但不真正解算的 shell 脚本)验证了进程监管、conf 生成与流
转发的"管道"是否正确,**没有用真实 rtkrcv 二进制验证过**:

- **conf 是否被真实 rtkrcv 接受**——`render_rtkrcv_conf()` 生成的键名
  (`pos1-posmode`、`pos1-elmask`、`pos2-armode`、`pos1-navsys`、`out-timesys`
  等)是否是真实 RTKLIB(demo5)认识的键名、是否还缺 Task 5 未覆盖到的必需
  字段,只能靠真机验证。
- **真实的 llh 解算内容与节奏**——`parse_llh_solution` 期望的列序是照文档和
  假设对齐的,真实 RTKLIB 在各种解质量(float/DGPS/单点)、丢星、AR 状态切换
  下实际吐出的行是否总能被正确解析,未验证。
- **真实 `$SAT`/`.stat` 文件格式与命名**——`-r 2` 参数下 rtkrcv 实际生成的
  文件名模式、是否会在长时间运行后滚动出多个文件、`plan_stat_tail()`
  按 mtime 取最新是否总能对上真实场景,未用真实二进制验证。
- **rtkrcv 对 SIGTERM 的真实响应**——`ProcessSupervisor` 的信号/超时升级逻辑
  本身已用假二进制验证过,但真实 rtkrcv 收到 SIGTERM 后是否会先 flush 完
  `.stat`/解算流再退出,未知。
- **崩溃循环退避在真实 conf 错误下的表现**——例如一个 rtkrcv 无法识别的 conf
  字段导致它秒退,`crash_loop_life_s` 退避是否如预期触发,未用真实二进制验证过
  (退避逻辑本身已用 `fake_rtkrcv.sh` 单测覆盖,这里只是没有用真实二进制复现过)。
- **多客户端/大流量下 `LocalReserver` 与 rtkrcv 的真实交互**——目前只验证了单个
  自制 TCP 客户端收到 `LocalReserver` 广播的字节;rtkrcv 作为 `tcpcli` 连入后的
  真实读取节奏、断线重连行为,未验证。
- **端到端**——`rtcm_bridge` → `rtkrcv_node` → `~/rtk_fix` 的全链路需要真实的
  差分流与观测流,现场设备到位前无法验证。

装上 demo5 版 RTKLIB 后,建议按上面七条逐一补验,而不是假定管道跑通了就等于
解算正确。

## 已知问题

- **`rtkrcv_node` 的 `args` 参数不能在 YAML 里写成空列表**——`args: []` 会让
  ROS2 的 YAML 参数加载器因为无法从空列表推断元素类型而在节点启动时抛
  `InvalidParameterValueException`,直接 `std::terminate()`(已实测复现)。
  当前的规避方式是 `config/gnss_bringup.yaml` 里干脆不写这个 key(节点侧
  `args` 参数本身默认值就是空列表,不写等价于"不传额外参数")。
  这只是绕开了症状,没有解决根因:任何人往这份 YAML 里加一行 `args: []`
  仍然会复现同样的崩溃,而且报错信息不会指向这份 README。正确的修复是在
  `rtkrcv_node.cpp` 声明该参数时,显式指定 `rcl_interfaces::msg::ParameterDescriptor`
  把类型固定为字符串数组(而不是依赖默认值推断类型),或者干脆换成一个用分隔符
  拼接的字符串参数(例如 `extra_args: "-s"`,按空格切分)。这不属于 Task 8 的
  修改范围(会改到 `rtkrcv_node.cpp`),记在这里留给后续任务处理。

## 不装 RTKLIB 也能跑起来:用 `test/fake_rtkrcv.sh`

`test/fake_rtkrcv.sh` 是一个不需要真实 RTKLIB 就能跑的替身二进制,足以练习整条
监管/转发管道(启动、conf 生成、TCP 转发、SIGTERM 收尾)。`launch.py` 里
`binary` 是写在 YAML 里的节点参数,不是 launch 参数,不能用
`ros2 launch ... key:=value` 直接覆盖;两种可行方式:

复制一份 `config/gnss_bringup.yaml`,把 `rtkrcv_node.binary` 改成
`fake_rtkrcv.sh` 的绝对路径、`args` 改成 `["live"]`,再用
`params_file:=<改过的路径>` 整份替换:

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py params_file:=/path/to/fake.yaml
```

或者不经过 launch 文件,直接单独跑 `rtkrcv_node` 验证监管/转发逻辑:

```bash
ros2 run gnss_bringup rtkrcv_node --ros-args \
  -p binary:="$(pwd)/src/gnss_bringup/test/fake_rtkrcv.sh" -p "args:=['live']" \
  -p run_dir:=/tmp/rtkrcv_smoke_run
```

或者只起 `rtcm_bridge`(完全不涉及 rtkrcv,不需要任何替身):

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py enable_rtkrcv:=false
```

配合 `nc` 手动灌一段字节验证桥接:

```bash
# rtcm_corrections 默认 listen=false,即 rtcm_bridge 主动连 127.0.0.1:15031;
# 用 nc -l 在该端口起一个假的差分源
nc -l 15031 < some_bytes.bin
ros2 topic echo /gnss/rtcm_corrections
```
