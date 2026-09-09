# gnss_core/tools —— 无 LiDAR 的轨迹对比与定权系数标定

spec §9(v2)。这些工具只依赖 `gnss_core`(纯 C++)与 Python 3,不需要 ROS、GLIM 或实车,
目的是把 `config_rtk_global.json` 里 `quality_sigma_scale` 的**工程估计值换成实测值**。

| 文件 | 用途 |
|---|---|
| `calibrate_sigma_scale.cpp` | `calibrate_sigma_scale <ref.pos> <test.pos> [tol_s]`:按 test 解质量分档统计 RMSE 与 "实际误差 / 板卡 σ" 三种比值,给出建议系数 |
| `export_bag_to_pos.py` | rtk-monitor 既有录包(SQLite `epochs` 表)→ 标准 `.pos`(轨迹 1–3:can / gpchc / rtkrcv);`--pos` 输入做时间系统/列规范化。测试:`python3 -m unittest tests/test_export_bag_to_pos.py` |
| `make_synthetic_pos.py` | 生成一对已知偏移的合成 `.pos`,用于自检工具本身 |
| `synth_rtk_fix.cpp` | GLIM 轨迹(TUM)→ 合成 RTK 观测(含四种故障注入)→ `.pos`(+ `--truth` 真值);spec §12.3,Task 13 |
| `pos_to_rtkfix_bag.py` | `.pos` → `gnss_msgs/RtkFix` rosbag2,`--merge` 与 LiDAR bag 归并成单包。**只能在 Orin 跑,未在无 ROS 环境验证** |
| `run_injection_suite.sh` | 四种注入 × huber/none 跑 `glim_rosbag` 并汇总 RMSE 表(Orin;含 `### CHECK` 标注的现场核对点) |
| `traj_rmse.py` | 两条 TUM 轨迹按时间配对算 RMSE(`evo_ape` 的回退,只依赖 numpy;支持 `--align`、时间窗) |
| `estimate_lever_arm.cpp` | GLIM 轨迹 + RTK `.pos` → `lever_imu`、`T_world_enu`、`time_offset`;spec §9.3,Task 14 |

## 1. 数据从哪来

四条轨迹同为 RTKLIB `.pos` 格式(spec §9.1):

| # | 轨迹 | 来源 | 时间系统 |
|---|---|---|---|
| 1 | 610 组合导航融合解 | `export_bag_to_pos.py --src can` | UTC(脚本写 `time=UTC` 头) |
| 2 | 610 卫导解 | `export_bag_to_pos.py --src gpchc` | UTC |
| 3 | rtkrcv 独立解算 | `export_bag_to_pos.py --src rtkrcv` | UTC |
| 4 | 后处理基准(参考真值) | `rnx2rtkp -p 3 -k conf.conf rover.obs base.obs nav -o ref.pos` | **GPST**(RTKLIB 默认;头部 `time=GPST`) |

`ref` 优先选 4;没有后处理时取质量最高的一条(通常 3)。

### `export_bag_to_pos.py --db`

```bash
export_bag_to_pos.py --db data/2026-09-03.db --src can    --out can.pos
export_bag_to_pos.py --db data/2026-09-03.db --src rtkrcv --out rtkrcv.pos --t0 1757300000 --t1 1757303600
export_bag_to_pos.py --db data/2026-09-03.db --src gpchc  --out gpchc.pos --keep-nofix
```

rtk-monitor 的 schema(`rtk_monitor/storage/epochs.py`):
`epochs(t REAL UTC unix 秒, src ∈{rtkrcv,gpchc,can}, q, sats, age, lat, lon, alt, sde, sdn, sdu, ratio, …)`。

- `q` 按 `src` 解释:`rtkrcv` 是 RTKLIB Q 原样透传;`gpchc`/`can` 是 CGI-610 `satellite_status`,映射
  4/8→fix(1)、5/9→float(2)、2/7→dgps(4)、1/6/3→single(5)、0/未知→无解。
- 无解历元默认**跳过**;`--keep-nofix` 保留并写 `Q=0`(`read_pos` 映射为 NONE;`compare_by_quality`
  默认 `ref_min_q=1` 时作为 ref 也会被忽略)。
- DB 的 σ 列顺序是 `sde sdn sdu`,输出按 `.pos` 约定写 `sdn sde sdu`;`sats/age/ratio/σ` 为 NULL 写 0。
- `--t0/--t1` 为 UTC unix 秒闭区间;输出头固定 `time=UTC`,按时间升序。

**时间系统**:`gnss_core::read_pos` 会读 `%` 头里的 `time=GPST` / `time=UTC` 并统一成 UTC unix 秒
(GPST 减 18 s 闰秒,可配)。**没有头部标注时按 GPST 处理**(RTKLIB 约定)。如果 `calibrate_sigma_scale`
报 `no epochs paired within tolerance`,九成是 test 文件缺头且实际是 UTC ——用
`export_bag_to_pos.py --pos in.pos --pos-time UTC --src <x> --out out.pos` 规范化后再跑。

## 2. 跑标定

```bash
cmake -S gnss_core -B build && cmake --build build -j
./build/calibrate_sigma_scale ref.pos rtkrcv.pos          # 默认配对容差 0.1 s
./build/calibrate_sigma_scale ref.pos can.pos 0.05        # 1 Hz 摘要一般 0.1 s 足够;高频数据可收紧
```

Orin 上 colcon 构建后可执行在 `install/gnss_core/lib/gnss_core/calibrate_sigma_scale`。

输出三种比值(spec §9.2 v2):

- `ratio_median` = median(err_h / σ_h) —— **建议系数取此**,对一次错误固定(σ=3 mm、误差 1 m → 比值 300)免疫
- `ratio_rms` = RMS(err_h) / RMS(σ_h) —— 与 RMSE 一致的口径,可交叉核对
- `ratio_mean` = mean(err_h / σ_h) —— 仅参考,受外点主导

其中 `σ_h = hypot(sdn, sde)`;板卡未报 σ(`sdn=sde=0`)的历元计入 `n` 与 RMSE,但不进比值。

### 合成数据自检(现场数据回来前)

```bash
python3 gnss_core/tools/make_synthetic_pos.py /tmp/synth      # 600 历元,FIXED/FLOAT/SINGLE 各 200
./build/calibrate_sigma_scale /tmp/synth/synth_ref.pos /tmp/synth/synth_test.pos [tol_s=0.1] [ref_min_q=1]
```

`ref_min_q`:只用 `Q<=ref_min_q` 且 `Q!=0` 的 ref 记录作基准(默认 1 = 仅 FIXED;传 0 不过滤)。
合成 ref 全是 FIXED,默认即可;真实 rnx2rtkp 基准含 FLOAT 段时默认会把这些段剔除。

**读数注意**:`ratio_median` 有偏——板卡 σ 完全可信时 `err_h/σ_h` 服从 Rayleigh(1/√2),中位数 ≈ 0.83
而不是 1;`ratio_rms` 期望才是 1(无偏),但受外点影响。两列对照看。

注入设定:FIXED 真实误差 std = 板卡 σ(σ 可信);FLOAT 真实误差是板卡 σ 的 3 倍;SINGLE 是 2 倍。
理论期望 `ratio_rms` = 1 / 3 / 2,`ratio_median` = 0.83 × 同值(瑞利中位数 1.177σ ÷ √2)。
实际输出(seed=1):

```
quality      n  rmse_h(m)  rmse_v(m)  ratio_median  ratio_rms  ratio_mean
SINGLE     200      2.811      2.930         1.751      1.987       1.802
FLOAT      200      0.436      0.430         2.513      3.080       2.732
FIXED      200      0.014      0.015         0.886      1.015       0.907

suggested quality_sigma_scale (median ratio, normalized to FIXED=1):
  SINGLE  = 1.977
  FLOAT   = 2.837
  FIXED   = 1.000

note: FIXED tier raw median ratio = 0.886; if far from 1, also scale sigma_floor / the board sigma globally by this amount.
```

与理论一致(FLOAT ≈ 3、SINGLE ≈ 2、FIXED 的 `ratio_rms` ≈ 1),工具本身正确。
**待现场 ref/test 成对数据回来后重跑,并把结果贴到本节下方替换合成示例。**

## 3. 把系数填回 `config_rtk_global.json`(spec §9.2 闭环)

`suggested quality_sigma_scale` 一节直接对应配置里的 `quality_sigma_scale`(按 `Quality` 枚举顺序
`NONE SINGLE DGPS FLOAT FIXED`;工具没统计到的档保留原值):

```json
"quality_sigma_scale": { "FIXED": 1.0, "FLOAT": 2.8, "DGPS": 5.0, "SINGLE": 2.0 }
```

两条注意:

1. 建议值已归一到 FIXED=1。若 FIXED 档**原始**中位数比值本身明显偏离 1(输出末行 `note`),说明板卡 σ 整体
   偏乐观/悲观,应同时把 `sigma_floor` 或全局 σ 放大该倍数,而不是只调分档系数。
2. 杆臂未标定前 `sigma_floor` 保持 1.0 m(spec §7.3 v2),分档系数在 floor 之上才起作用;
   用 `estimate_lever_arm`(Task 14)标定杆臂后再把 floor 降到 cm 级。

## 4. RTKPLOT 可视化对比

RTKPLOT 可直接叠加多条 `.pos`(`.pos` 格式的主要理由之一):

```bash
rtkplot ref.pos rtkrcv.pos can.pos gpchc.pos       # 依次打开,Solution 1/2 可切换/相减
# 或 GUI:File → Open Solution-1 选 ref.pos,Open Solution-2 选 test.pos,
#        View → Plot Type → "Position"/"Ground Track",Solution → "Sol-1 - Sol-2" 看差值
```

RTKPLOT 默认按 GPST 显示时间;`export_bag_to_pos.py` 写出的是 `time=UTC` 头,RTKPLOT 会自行识别。

## 5. 合成注入(Task 13)与杆臂/时间偏移估计(Task 14)

```bash
# 合成:0–20 s 直行、转 90°、再直行的轨迹见 test/synth_fixtures.hpp;实际用 glim_rosbag dump 的 traj_imu.txt
./build/synth_rtk_fix traj_imu.txt rtk.pos --lever 0 1 0 --wrong-fix 0.05 --stale +10 +20 --float +30 +40 \
    --truth truth.pos --seed 42          # '+N' = 相对轨迹起点的秒数
# 估计杆臂/时间偏移(合成数据自检;真实数据用同段 traj_imu.txt + export_bag_to_pos.py 的 .pos)
./build/estimate_lever_arm traj_imu.txt rtk.pos --origin 44.5 90.28 617
# 整套注入(Orin):
tools/run_injection_suite.sh <lidar_bag_dir> /tmp/inj_suite
```

`estimate_lever_arm` 输出末尾两行可直接粘进 `config_rtk_global.json`;轨迹 yaw 极差 < 30° 时会打印
`lever NOT observable`,此时水平杆臂只能量取。

## 6. 现场待办

- [x] `export_bag_to_pos.py::load_epochs_from_db` 按 rtk-monitor 实际 SQLite schema 实现(见 §1)
- [ ] 在真实 `.db` 上跑一次 `--src can/gpchc/rtkrcv`,核对 CGI-610 状态码映射与 `t` 的时间系统(脚本假定 UTC unix 秒)
- [ ] 拿到一段有 rnx2rtkp 后处理基准的数据,跑 `calibrate_sigma_scale ref.pos rtkrcv.pos`,把输出替换本 README §2 的合成示例
- [ ] 结果写入 `glim_ext/config/config_rtk_global.json` 的 `quality_sigma_scale`
- [ ] Orin 上首次运行 `pos_to_rtkfix_bag.py`(短 `.pos` 试跑 + `ros2 bag info` 核对),再跑 `run_injection_suite.sh`,把 `summary.txt` 贴到本 README
- [ ] 实车 LiDAR+IMU bag 无 GNSS 跑出 `traj_imu.txt` + 同段 `RtkFix` 导出 `.pos`,跑 `estimate_lever_arm`,结果填入 config 并把 `sigma_floor` 降到 `[0.05,0.05,0.1]`
