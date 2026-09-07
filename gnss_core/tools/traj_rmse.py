#!/usr/bin/env python3
"""traj_rmse.py —— 两条 TUM 轨迹按时间配对算位置 RMSE(evo_ape 不可用时的回退;Task 13 Step 6)

usage: traj_rmse.py <ref.txt> <est.txt> [--tol 0.05] [--from T] [--to T] [--align]
                    [--relative-time]

- TUM 格式:t x y z qx qy qz qw;'#' 行跳过。
- 配对:对 est 的每个时刻,在 ref 中找最近时刻(容差 --tol,默认 0.05 s);ref 侧线性插值位置。
- --from/--to:只统计该时间段(绝对时间;--relative-time 时为相对 ref 起点的秒数),用于看"转弯段"。
- --align:先用 Umeyama(仅刚体,不缩放)把 est 对齐到 ref 再算——两条轨迹 gauge 不同(比如
  有 GNSS 与无 GNSS 的 GLIM 结果)时必须开;同一 gauge 下不开更能反映绝对偏移。
- 只依赖 numpy;输出一行:n rmse mean median max(m),便于 shell 汇总。
"""
import argparse
import sys

import numpy as np


def read_tum(path):
    rows = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            c = s.split()
            if len(c) < 4:
                continue
            rows.append([float(v) for v in c[:4]])
    if not rows:
        raise SystemExit(f"no poses in {path}")
    a = np.array(rows)
    a = a[np.argsort(a[:, 0], kind="stable")]
    return a[:, 0], a[:, 1:4]


def pair(t_ref, p_ref, t_est, p_est, tol):
    """按 est 时刻在 ref 上线性插值;超出 ref 范围或最近点间隔 > tol 的丢弃。"""
    keep = []
    q = []
    for i, t in enumerate(t_est):
        if t < t_ref[0] or t > t_ref[-1]:
            continue
        j = np.searchsorted(t_ref, t)
        j0 = max(j - 1, 0)
        j1 = min(j, len(t_ref) - 1)
        if min(abs(t_ref[j0] - t), abs(t_ref[j1] - t)) > tol:
            continue
        span = t_ref[j1] - t_ref[j0]
        s = (t - t_ref[j0]) / span if span > 0 else 0.0
        q.append((1 - s) * p_ref[j0] + s * p_ref[j1])
        keep.append(i)
    return np.array(q), p_est[keep], t_est[keep]


def umeyama_rigid(src, dst):
    """R, t 使 R·src + t ≈ dst(无缩放)。"""
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(Vt.T @ U.T) < 0:
        D[2, 2] = -1
    R = Vt.T @ D @ U.T
    return R, md - R @ ms


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ref")
    ap.add_argument("est")
    ap.add_argument("--tol", type=float, default=0.05)
    ap.add_argument("--from", dest="t_from", type=float, default=None)
    ap.add_argument("--to", dest="t_to", type=float, default=None)
    ap.add_argument("--relative-time", action="store_true")
    ap.add_argument("--align", action="store_true")
    args = ap.parse_args()

    t_ref, p_ref = read_tum(args.ref)
    t_est, p_est = read_tum(args.est)
    ref_pts, est_pts, t_pair = pair(t_ref, p_ref, t_est, p_est, args.tol)
    if len(t_pair) == 0:
        print("n=0 (no pairs within tol)")
        return 1

    if args.align:
        R, t = umeyama_rigid(est_pts, ref_pts)
        est_pts = (R @ est_pts.T).T + t

    t0 = t_ref[0] if args.relative_time else 0.0
    mask = np.ones(len(t_pair), dtype=bool)
    if args.t_from is not None:
        mask &= t_pair >= args.t_from + t0
    if args.t_to is not None:
        mask &= t_pair <= args.t_to + t0
    err = np.linalg.norm(ref_pts[mask] - est_pts[mask], axis=1)
    if err.size == 0:
        print("n=0 (no pairs in time window)")
        return 1
    print(f"n={err.size} rmse={np.sqrt(np.mean(err ** 2)):.4f} mean={err.mean():.4f} "
          f"median={np.median(err):.4f} max={err.max():.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
