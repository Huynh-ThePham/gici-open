#!/usr/bin/env python3
"""Diagnose UrbanNav Medium eval gap: horizontal vs 3D APE and attitude."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
from pathlib import Path
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def load_baseline_module():
    from importlib.machinery import SourceFileLoader

    return SourceFileLoader(
        "urb", str(REPO / "scripts" / "run_urbannav_rrr_baseline.py")
    ).load_module()


def parse_evo_rmse(text: str) -> float:
    m = re.search(r"^\s*rmse\s+([0-9.]+)\s*$", text, re.MULTILINE)
    if not m:
        raise RuntimeError(f"Could not parse evo rmse from:\n{text}")
    return float(m.group(1))


def evo_rmse(gt_tum: Path, est_tum: Path, *, rotation: bool = False, xy: bool = False) -> float:
    cmd = [
        "evo_ape", "tum", str(gt_tum), str(est_tum),
        "-va", "--align", "--correct_scale",
    ]
    if rotation:
        cmd += ["--pose_relation", "angle_deg"]
    if xy:
        cmd += ["--project_to_plane", "xy"]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
    return parse_evo_rmse(proc.stdout)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", type=Path,
                    default=REPO / "results/baseline/urbannav/medium")
    args = ap.parse_args()

    mod = load_baseline_module()
    data_root = Path(os.environ.get(
        "URBANNAV_DATA_ROOT",
        "/media/theph/Data1/Research/dataset/UrbanNavDataset",
    ))
    root = data_root / "UrbanNav-HK-Medium-Urban-1"
    gt = mod.load_ground_truth(root / "UrbanNav_TST_GT_raw.txt")
    solution = args.out_dir / "output/solution.txt"
    eval_dir = args.out_dir / "evaluation"
    gt_tum = eval_dir / "trajectory_gt.tum"
    est_tum = eval_dir / "trajectory_est.tum"

    epochs = mod.parse_solution(solution, 86400.0)
    ref_lat, ref_lon, ref_h = gt["llh"][0]
    ref_ecef = mod.llh_to_ecef(ref_lat, ref_lon, ref_h)

    e_vals, n_vals, u_vals = [], [], []
    for ep in epochs:
        gpos = mod.interp_llh(gt["times"], gt["llh"], ep["gpst"])
        if gpos is None:
            continue
        se = mod.ecef_to_enu(
            *mod.llh_to_ecef(ep["lat"], ep["lon"], ep["h"]),
            ref_ecef, ref_lat, ref_lon,
        )
        ge = mod.ecef_to_enu(
            *mod.llh_to_ecef(*gpos), ref_ecef, ref_lat, ref_lon,
        )
        e_vals.append(se[0] - ge[0])
        n_vals.append(se[1] - ge[1])
        u_vals.append(se[2] - ge[2])

    def rmse(vals: list[float]) -> float:
        return math.sqrt(sum(v * v for v in vals) / len(vals))

    report: dict[str, Any] = {
        "dataset": "urbannav_medium",
        "tst_rmse_h_m": rmse([math.hypot(e, n) for e, n in zip(e_vals, n_vals)]),
        "tst_rmse_u_m": rmse(u_vals),
        "tst_u_bias_mean_m": sum(u_vals) / len(u_vals),
        "paper_reference": {"ape_position_m": 3.40, "ape_rotation_deg": 1.30},
    }

    if gt_tum.is_file() and est_tum.is_file():
        report["evo_ape_3d_translation_rmse_m"] = evo_rmse(gt_tum, est_tum)
        report["evo_ape_3d_rotation_rmse_deg"] = evo_rmse(gt_tum, est_tum, rotation=True)
        report["evo_ape_xy_translation_rmse_m"] = evo_rmse(gt_tum, est_tum, xy=True)
        report["evo_ape_xy_rotation_rmse_deg"] = evo_rmse(gt_tum, est_tum, rotation=True, xy=True)

    report["interpretation"] = {
        "diagnostic_only": True,
        "position_note": (
            "Author APE = evo_ape 3D Sim(3) on full solution (README §4). "
            "Medium early vertical bias inflates 3D APE; see tst_rmse_h / tst_rmse_u for diagnosis."
        ),
        "vertical_bias_m": report.get("tst_u_bias_mean_m"),
        "rotation_note": (
            "Pass criteria use locked evo_ape in expected_urbannav_medium.json "
            "(same logic as run_author_eval_1_1.sh)."
        ),
    }

    out = eval_dir / "medium_eval_diagnosis.json"
    out.write_text(json.dumps(report, indent=2) + "\n")

    print(json.dumps(report, indent=2))
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
