#!/usr/bin/env python3
"""F4: horizontal ENU error over time — VA-v4 run3 (tail) vs baseline, fix markers."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import run_urbannav_rrr_baseline as R  # noqa: E402

OUT = REPO / "research/paper/fig/F4_tail.pdf"
VA = REPO / "results/research/urbannav_va4solo_20260718/run3/deep/output/solution.txt"
BL = REPO / "results/research/paper_repro/deep/bl/run3/deep/output/solution.txt"


def series(sol: Path, gt: dict, off: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    epochs = R.parse_solution(sol, off)
    ref_lat, ref_lon, ref_h = gt["llh"][0]
    ref_ecef = R.llh_to_ecef(ref_lat, ref_lon, ref_h)
    t0 = None
    ts, hs, fixed = [], [], []
    for ep in epochs:
        gt_pos = R.interp_llh(gt["times"], gt["llh"], ep["gpst"])
        if gt_pos is None:
            continue
        if t0 is None:
            t0 = ep["gpst"]
        se = R.ecef_to_enu(*R.llh_to_ecef(ep["lat"], ep["lon"], ep["h"]), ref_ecef, ref_lat, ref_lon)
        ge = R.ecef_to_enu(*R.llh_to_ecef(*gt_pos), ref_ecef, ref_lat, ref_lon)
        ts.append(ep["gpst"] - t0)
        hs.append(float(np.hypot(se[0] - ge[0], se[1] - ge[1])))
        # quality 4 = fixed in GICI NMEA convention used by runner
        fixed.append(1 if ep.get("quality") == 4 or ep.get("fix") else 0)
    return np.asarray(ts), np.asarray(hs), np.asarray(fixed)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--va", type=Path, default=VA)
    ap.add_argument("--bl", type=Path, default=BL)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    ds = R.DATASETS["deep"]
    gt = R.load_ground_truth(ds.root / ds.gt_file)
    off = ds.gps_week_day_offset

    # detect fix flag field
    sample = R.parse_solution(args.va, off)[:5]
    # patch series to use quality from parse_solution
    def series2(sol: Path):
        epochs = R.parse_solution(sol, off)
        ref_lat, ref_lon, ref_h = gt["llh"][0]
        ref_ecef = R.llh_to_ecef(ref_lat, ref_lon, ref_h)
        t0 = epochs[0]["gpst"]
        ts, hs, fx = [], [], []
        for ep in epochs:
            gt_pos = R.interp_llh(gt["times"], gt["llh"], ep["gpst"])
            if gt_pos is None:
                continue
            se = R.ecef_to_enu(*R.llh_to_ecef(ep["lat"], ep["lon"], ep["h"]), ref_ecef, ref_lat, ref_lon)
            ge = R.ecef_to_enu(*R.llh_to_ecef(*gt_pos), ref_ecef, ref_lat, ref_lon)
            ts.append(ep["gpst"] - t0)
            hs.append(float(np.hypot(se[0] - ge[0], se[1] - ge[1])))
            q = ep.get("quality", ep.get("q", 5))
            fx.append(int(q) == 4)
        return np.asarray(ts), np.asarray(hs), np.asarray(fx)

    print("sample keys", sample[0].keys() if sample else None)
    t_va, h_va, f_va = series2(args.va)
    t_bl, h_bl, _ = series2(args.bl)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(5.2, 2.8))
    ax.plot(t_bl / 60.0, h_bl, color="#888888", lw=0.7, label="baseline")
    ax.plot(t_va / 60.0, h_va, color="#b00020", lw=0.8, label="VA-v4 run3")
    if f_va.any():
        ax.scatter(t_va[f_va] / 60.0, h_va[f_va], s=10, c="#1f4e79", zorder=3, label="VA fixed")
    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Horizontal error (m)")
    ax.set_ylim(0, min(40, float(np.percentile(h_va, 99.5)) * 1.2))
    ax.legend(frameon=False, fontsize=8, loc="upper right")
    ax.set_title("Deep horizontal error (tail episode)")
    fig.tight_layout()
    fig.savefig(args.out)
    print(f"wrote {args.out}  va_n={len(h_va)} fixed={int(f_va.sum())} max_h={h_va.max():.1f}")


if __name__ == "__main__":
    main()
