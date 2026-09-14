# 回放测试数据

`test_rtkrcv_real_binary` 用的两路 RTCM3,由 RTKLIB-EX 2.5.1 源码树自带的 GSI 两站 RINEX
(`test/data/rinex/07590920.05o`、`30400920.05o`,2005-04-02 00:00–00:59:30 GPST,30 s 间隔,
基线约 3.3 km;RTKLIB 以 BSD-2-Clause 发布)经 `rnx2rtcm.c` 转换:

| 文件 | 测站 | 用途 |
|---|---|---|
| `rtcm_20050402_0759_rover.rtcm3` | 0759 | 流动站原始观测(喂 `/gnss/raw_obs`) |
| `rtcm_20050402_3040_base.rtcm3`  | 3040 | 差分(喂 `/gnss/rtcm_corrections`,含 1005 基准站坐标) |

每个历元一条 1004,每 10 个历元补发 GPS 星历 1019 与 1005。转换丢了锁定时间信息,
rtkrcv 只能得到浮点解(原始 RINEX 后处理可以固定)——这份数据验证的是链路与解析,不是固定率。

重新生成的命令与期望的 sha256 见 `docs/gnss/plans/2026-09-14-round2-hardening.md` Task 6 Step 1。

回放必须用 libfaketime 把 rtkrcv 的时钟拨到 2005-04-02 那一周:RTCM MSM/1004 只带周内秒,
rtkrcv 用系统时间补周数,补错了星历对不上、一条解都没有。
