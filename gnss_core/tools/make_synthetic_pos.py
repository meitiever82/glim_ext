#!/usr/bin/env python3
"""生成一对已知偏移的合成 .pos(ref / test),用于自检 calibrate_sigma_scale。

test 相对 ref 的注入误差(水平,高斯)与板卡报 σ 按质量档设定,因此期望的
"实际误差 / σ_h" 中位数比值是已知的:

  档     σ_n=σ_e   真实误差 std(每轴)  期望 median(err_h/σ_h)   期望 ratio_rms
  FIXED   0.01 m    0.01 m              ≈ 0.83                    1.0  (板卡 σ 可信)
  FLOAT   0.10 m    0.30 m              ≈ 2.50                    3.0  (板卡低估 3 倍)
  SINGLE  1.00 m    2.00 m              ≈ 1.66                    2.0

(瑞利分布中位数 = 1.177 σ;err_h/σ_h 中 σ_h = hypot(σ,σ) = 1.414 σ,故
 median = 1.177*err_std / (1.414*σ) = 0.83 * err_std/σ;RMS(err_h)/RMS(σ_h) = err_std/σ。)
归一到 FIXED=1 后建议系数 ≈ FLOAT 3.0、SINGLE 2.0。

用法:make_synthetic_pos.py <out_dir> [--n 600] [--seed 1]
输出 <out_dir>/synth_ref.pos、<out_dir>/synth_test.pos(GPST 头,与 RTKLIB 一致)。
"""
import argparse
import math
import os
import random
import time

LEAP = 18
DEG_PER_M_LAT = 1.0 / 111320.0


def fmt_time(unix_utc):
    t = unix_utc + LEAP  # GPST
    whole = math.floor(t)
    ms = int(round((t - whole) * 1000))
    if ms >= 1000:
        ms -= 1000
        whole += 1
    tm = time.gmtime(whole)
    return "%04d/%02d/%02d %02d:%02d:%02d.%03d" % (tm.tm_year, tm.tm_mon, tm.tm_mday,
                                                   tm.tm_hour, tm.tm_min, tm.tm_sec, ms)


HEADER = ("% program   : make_synthetic_pos.py\n"
          "% (lat/lon/height=WGS84/ellipsoidal,Q=1:fix,2:float,4:dgps,5:single, time=GPST)\n"
          "%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)"
          "  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n")


def line(t, lat, lon, h, q, ns, sd, age, ratio):
    return "%s %14.9f %14.9f %10.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n" % (
        fmt_time(t), lat, lon, h, q, ns, sd, sd, sd * 2, 0, 0, 0, age, ratio)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--n", type=int, default=600)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    random.seed(a.seed)
    os.makedirs(a.out_dir, exist_ok=True)

    lat0, lon0, h0 = 44.5, 90.28, 617.0
    deg_per_m_lon = DEG_PER_M_LAT / math.cos(math.radians(lat0))
    t0 = 1788431025.0
    # (Q, sigma_board, true_err_std)
    tiers = [(1, 0.01, 0.01), (2, 0.10, 0.30), (5, 1.00, 2.00)]

    with open(os.path.join(a.out_dir, "synth_ref.pos"), "w") as fr, \
         open(os.path.join(a.out_dir, "synth_test.pos"), "w") as ft:
        fr.write(HEADER)
        ft.write(HEADER)
        for i in range(a.n):
            t = t0 + i
            # 参考轨迹:沿东北方向匀速 1 m/s 的直线,加轻微起伏
            lat = lat0 + 0.7 * i * DEG_PER_M_LAT
            lon = lon0 + 0.7 * i * deg_per_m_lon
            h = h0 + 0.2 * math.sin(i / 30.0)
            fr.write(line(t, lat, lon, h, 1, 40, 0.005, 0.5, 50.0))
            q, sd, err = tiers[(i // 50) % len(tiers)]
            dn = random.gauss(0, err)
            de = random.gauss(0, err)
            du = random.gauss(0, err * 1.5)
            ft.write(line(t, lat + dn * DEG_PER_M_LAT, lon + de * deg_per_m_lon, h + du,
                          q, 32, sd, 1.0, 3.0 if q == 1 else 1.5))
    print("wrote", os.path.join(a.out_dir, "synth_ref.pos"), os.path.join(a.out_dir, "synth_test.pos"))


if __name__ == "__main__":
    main()
