# 本地 RTCM 回放 Demo —— 无车点亮 rtkrcv 解算链路 + 天空图

用一对录制/公开的 RTCM3 数据把 **rtkrcv 独立解算 → 轨迹 → 状态 → 天空图 →
诊断规则链**整条链路在本机跑起来,不需要车、不需要真板卡。适合联调前验证、
演示、以及回归排查。

> 这条链路里 `status.sol` 会出现真实的固定/浮动解,天空图会显示每颗卫星
> (来自 rtkrcv `-r 2` 的 `$SAT` 状态流)。静态测站数据出的是一个厘米级
> 固定"点"而非移动轨迹;移动轨迹需现场 1Hz 板卡实录。

## 需要准备

1. **rtkrcv 二进制**:`tools/bin/rtkrcv`(gitignored)。构建/提取见
   [integration-rtkrcv.md](integration-rtkrcv.md)。ARM64 可从交付镜像里提取,
   x86 可用 RTKLIB demo5 自行编译。
2. **一对短基线 CORS 观测 + 广播星历**(RINEX3):
   - 基站观测 `SITEA…_MO`(含头里的精确坐标)
   - 流动站观测 `SITEB…_MO`(离 A 近,基线 <10~20km)
   - 当天混合广播星历 `BRDC…_MN`(GPS/GLO/GAL/BDS)
   来源:GNSS-Go、IGS/EUREF/US-CORS 等。同一天、时间重叠。
3. **RTKLIB 工具**:`rnx2rtcm`(RINEX→RTCM3)、`rnx2rtkp`(后处理验证,可选)。
4. **Hatanaka 解压**:`.crx` 需先转 `.rnx`。`uv pip install --python .venv hatanaka`
   然后 `python -c "import hatanaka,pathlib as P; P.Path('x.rnx').write_bytes(hatanaka.decompress(P.Path('x.crx').read_bytes()))"`。

## 步骤

### 1. 解压 + 转两路 RTCM3

```bash
# Hatanaka .crx -> .rnx（对两个观测文件各做一次；星历 .rnx 若是 .gz 先 gunzip）
# 基站：观测 + 站坐标(1006)
rnx2rtcm -sta 0 -typ 1006,1077,1087,1097,1127 -out base.rtcm3  SITEA…_MO.rnx
# 流动站：观测 + 星历(1019/1020/1042/1046) —— 星历这一环缺了 rtkrcv 会 "no navigation data"
rnx2rtcm -sta 1 -typ 1077,1087,1097,1127,1019,1020,1042,1046 -out rover.rtcm3 \
    SITEB…_MO.rnx  BRDC…_MN.rnx
```

截一小时窗口加 `-ts y/m/d h:m:s -te y/m/d h:m:s`。

### 2.（可选)后处理验证数据能解

```bash
rnx2rtkp -p 2 -m 10 -o pp.pos  SITEB…_MO.rnx SITEA…_MO.rnx BRDC…_MN.rnx
# 看 pp.pos 第 6 列 Q：1=固定 2=浮动 5=单点。有一定比例的 1 就说明数据 OK。
```

### 3. 起时间同步喂给器 + app

一键（推荐）——`DATA` 是含 `base.rtcm3`/`rover.rtcm3` 的目录：

```bash
tools/run_replay.sh DATA 25          # 停旧进程 → 起喂给器 → 起 app（web :8083）
```

或手动分开：

```bash
# 喂给器：base→6201, rover→6202，30x 加速（一小时 ~2 分钟）
python tools/rtkfeed_timed.py base.rtcm3 6201 rover.rtcm3 6202 30 &

# app：用回放示例配置（rtkrcv 已启用、本地端口、web 8083）
python -m rtk_monitor.main config.replay.example.yaml
```

浏览器打开 **http://127.0.0.1:8083**:轨迹(Esri 卫星底图,本地无瓦片时自动
回退)、状态条(固定/σ/星数/龄期)、天空图(按信噪比着色的卫星点)、事件流。
数据流进库后也可用页面上的**回放条**回看任意时间窗。

## 为什么用 `rtkfeed_timed.py` 而不是 `replay_sources.py`

`tools/replay_sources.py` 按**固定字节块**匀速推送。但 rover.rtcm3 里约 90% 是
星历消息,按字节比例 ≠ 按时间比例,会让 base/rover 的 GPST 错开超过 rtkrcv 的
`maxage`(默认 30s),结果只出个位数解。`tools/rtkfeed_timed.py` 解析每帧的
GPS MSM(1077)历元时标,把两路按同一 GPST 时间轴同步释放,rtkrcv 才能正常固定。

## 常见问题

- **rtkrcv 一直不出解 / "no navigation data"**:rover.rtcm3 没带星历
  (`-typ` 里漏了 1019/1020/1042/1046),或用了 `replay_sources.py` 导致 base/rover
  时间错开。用本文的 `-typ` 和 `rtkfeed_timed.py`。
- **一个解都没有、库里 rtkrcv=0**:确认只有一个 rtkrcv 进程。旧 app 的 rtkrcv
  子进程连的是预留端口(16010/16011),按 web 端口杀会漏掉它,两个 rtkrcv 抢
  sol 端口会让新进程写不出 `.stat`。用 `pkill -f rtk_monitor.main` 后再核对
  `pgrep -f tools/bin/rtkrcv` 只剩一个。
- **状态全是"过期/无解"**:回放的是历史数据,GPST 落后墙钟,诊断新鲜度门限会
  判过期。`config.replay.example.yaml` 已把 `diagnosis.sol_stale_s` 放大;上车时
  务必改回 spec 默认。
- **跑单元测试前**:先停掉本 app —— 它占用 8083/6201/6202/16010/16011/16020 等
  固定端口,测试要绑同端口会卡住(非代码缺陷)。
