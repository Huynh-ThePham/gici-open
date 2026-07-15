#!/usr/bin/env python3
"""DIAGNOSTIC ONLY — do not use for pass/fail or paper Table V.

Optional analysis: estimate when vertical error vs TST GT stabilizes.
Author APE in run_author_eval_urbannav.sh uses the full solution trajectory.
"""

from __future__ import annotations

import argparse
import math
from importlib.machinery import SourceFileLoader
from pathlib import Path


def rolling_rmse_u(
    gpst: list[float],
    u_err: list[float],
    *,
    window_s: float,
    end_idx: int,
) -> float:
    t_end = gpst[end_idx]
    t_start = t_end - window_s
    vals = [u for t, u in zip(gpst, u_err) if t_start <= t <= t_end]
    if len(vals) < 3:
        return float("inf")
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def detect_start_gpst(
    solution: Path,
    gt_raw: Path,
    gps_week_day_offset: float,
    *,
    window_s: float = 120.0,
    sustain_s: float = 90.0,
    threshold_m: float = 2.5,
) -> dict:
    repo = Path(__file__).resolve().parents[1]
    mod = SourceFileLoader("urb", str(repo / "scripts" / "run_urbannav_rrr_baseline.py")).load_module()
    gt = mod.load_ground_truth(gt_raw)
    epochs = mod.parse_solution(solution, gps_week_day_offset)
    ref_lat, ref_lon, ref_h = gt["llh"][0]
    ref_ecef = mod.llh_to_ecef(ref_lat, ref_lon, ref_h)

    gpst_list: list[float] = []
    u_err: list[float] = []
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
        gpst_list.append(ep["gpst"])
        u_err.append(se[2] - ge[2])

    if len(gpst_list) < 10:
        raise RuntimeError("Too few matched epochs for convergence detection")

    sustain_steps = max(3, int(sustain_s / max(0.1, gpst_list[1] - gpst_list[0])))
    start_gpst = gpst_list[0]
    for i in range(len(gpst_list)):
        if rolling_rmse_u(gpst_list, u_err, window_s=window_s, end_idx=i) > threshold_m:
            continue
        ok = True
        for j in range(i, min(len(gpst_list), i + sustain_steps)):
            if rolling_rmse_u(gpst_list, u_err, window_s=window_s, end_idx=j) > threshold_m:
                ok = False
                break
        if ok:
            start_gpst = gpst_list[i]
            break

    skip_s = max(0.0, start_gpst - gpst_list[0])
    return {
        "start_gpst": start_gpst,
        "skip_s_from_first_solution": skip_s,
        "matched_epochs": len(gpst_list),
        "window_s": window_s,
        "sustain_s": sustain_s,
        "threshold_m": threshold_m,
    }


def trim_nmea(solution: Path, out: Path, start_gpst: float, gps_week_day_offset: float) -> int:
    repo = Path(__file__).resolve().parents[1]
    mod = SourceFileLoader("urb", str(repo / "scripts" / "run_urbannav_rrr_baseline.py")).load_module()
    kept: list[str] = []
    for line in solution.read_text(errors="ignore").splitlines():
        if line.startswith("$GPGGA,"):
            parts = line.split(",")
            if len(parts) < 2 or not parts[1]:
                continue
            try:
                gpst = gps_week_day_offset + mod.parse_nmea_time(parts[1]) + 18.0
            except ValueError:
                continue
            if gpst < start_gpst:
                continue
        kept.append(line)
    out.write_text("\n".join(kept) + ("\n" if kept else ""))
    return sum(1 for ln in kept if "GPGGA" in ln)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("solution")
    ap.add_argument("gt_raw")
    ap.add_argument("--gps-week-day-offset", type=float, required=True)
    ap.add_argument("--out-trimmed")
    ap.add_argument("--out-json")
    args = ap.parse_args()

    solution = Path(args.solution)
    gt_raw = Path(args.gt_raw)
    info = detect_start_gpst(
        solution, gt_raw, args.gps_week_day_offset,
    )
    if args.out_json:
        import json
        Path(args.out_json).write_text(json.dumps(info, indent=2) + "\n")

    if args.out_trimmed:
        n = trim_nmea(
            solution, Path(args.out_trimmed), info["start_gpst"], args.gps_week_day_offset,
        )
        info["trimmed_gpgga_epochs"] = n
        if args.out_json:
            import json
            Path(args.out_json).write_text(json.dumps(info, indent=2) + "\n")
        print(f"Trimmed {solution} -> {args.out_trimmed}: {n} GPGGA from gpst={info['start_gpst']:.3f}")

    print(info)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
