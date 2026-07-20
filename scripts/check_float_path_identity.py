#!/usr/bin/env python3
"""Compare two solution.txt trajectories for float-path identity (0-fix isolation).

Pass criteria (suggested):
  - both arms report ~0% NlFix (quality==4) over matched epochs
  - horizontal ENU RMSE between arms << 1 cm median / << 5 cm p95
    (numerical noise), else float-path is NOT isolated.

Usage:
  python3 scripts/check_float_path_identity.py \\
    --a results/.../bl/solution.txt \\
    --b results/.../va_iso/solution.txt \\
    --gt /path/to/UrbanNav_TST_GT_cluster.txt
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import run_urbannav_rrr_baseline as R  # noqa: E402


def load_enu(sol: Path, gt: dict, off: float):
    epochs = R.parse_solution(sol, off)
    ref_lat, ref_lon, ref_h = gt["llh"][0]
    ref_ecef = R.llh_to_ecef(ref_lat, ref_lon, ref_h)
    rows = []
    for ep in epochs:
        gt_pos = R.interp_llh(gt["times"], gt["llh"], ep["gpst"])
        if gt_pos is None:
            continue
        se = R.ecef_to_enu(*R.llh_to_ecef(ep["lat"], ep["lon"], ep["h"]), ref_ecef, ref_lat, ref_lon)
        q = int(ep.get("quality", ep.get("q", 5)))
        rows.append((ep["gpst"], se[0], se[1], se[2], q == 4))
    return np.asarray(rows, dtype=float) if rows else np.zeros((0, 5))


def match(a: np.ndarray, b: np.ndarray, tol: float = 0.05):
    """Nearest-neighbor match on gpst within tol seconds."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((0, 3)), 0, 0
    jb = 0
    de, dn, du = [], [], []
    n_fix_a = n_fix_b = 0
    for row in a:
        t, e, n, u, fx = row
        n_fix_a += int(fx)
        while jb + 1 < len(b) and abs(b[jb + 1, 0] - t) < abs(b[jb, 0] - t):
            jb += 1
        if abs(b[jb, 0] - t) > tol:
            continue
        de.append(e - b[jb, 1])
        dn.append(n - b[jb, 2])
        du.append(u - b[jb, 3])
        n_fix_b += int(b[jb, 4])
    d = np.column_stack([de, dn, du]) if de else np.zeros((0, 3))
    return d, n_fix_a, n_fix_b


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", type=Path, required=True, help="baseline solution.txt")
    ap.add_argument("--b", type=Path, required=True, help="VA-iso solution.txt")
    ap.add_argument("--dataset", choices=["medium", "deep"], default="medium")
    ap.add_argument("--tol-s", type=float, default=0.05)
    args = ap.parse_args()

    ds = R.DATASETS[args.dataset]
    gt = R.load_ground_truth(ds.root / ds.gt_file)
    off = ds.gps_week_day_offset

    A = load_enu(args.a, gt, off)
    B = load_enu(args.b, gt, off)
    d, nfa, nfb = match(A, B, args.tol_s)
    if len(d) == 0:
        raise SystemExit("no matched epochs")

    h = np.hypot(d[:, 0], d[:, 1])
    print(f"matched={len(d)}  A_epochs={len(A)} B_epochs={len(B)}")
    print(f"NlFix count (quality==4): A={nfa} ({100*nfa/len(A):.2f}%)  "
          f"B={nfb} ({100*nfb/len(B):.2f}%)")
    print(f"arm-to-arm Δh: median={np.median(h):.4f} m  p95={np.percentile(h,95):.4f} m  "
          f"max={h.max():.4f} m  rmse={np.sqrt((h**2).mean()):.4f} m")
    print(f"arm-to-arm Δu: median={np.median(np.abs(d[:,2])):.4f} m  "
          f"rmse={np.sqrt((d[:,2]**2).mean()):.4f} m")

    # Soft pass: both ~0% fixed and median Δh < 1 cm under deterministic solver
    ok_fix = (nfa / len(A) < 0.001) and (nfb / len(B) < 0.001)
    ok_traj = np.median(h) < 0.01 and np.percentile(h, 95) < 0.05
    print("PASS_FIX_ZERO" if ok_fix else "FAIL_FIX_ZERO")
    print("PASS_TRAJ_IDENTITY" if ok_traj else "FAIL_TRAJ_IDENTITY")
    if not ok_traj:
        print("→ float path NOT isolated; see SUBMISSION_GAPS.md (WlFix? solver time? Evaluate side effects)")


if __name__ == "__main__":
    main()
