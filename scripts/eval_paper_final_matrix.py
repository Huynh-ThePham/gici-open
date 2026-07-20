#!/usr/bin/env python3
"""Aggregate + compare the paper final matrix (3 datasets × baseline/proposed × n=3).

Reads <OUT>/rep{1,2,3}/{bl,va4}/<ds>/metrics.json and reports, per dataset:
  - baseline vs proposed: mean±std over the 3 repeats for the key metrics,
  - paired deltas (proposed − baseline per repeat) with a sign count.

Metrics: rmse_h_m (horizontal), rmse_u_m (vertical), yaw_rmse_deg, fixed_rate.
Writes <OUT>/final_comparison.json and prints a table. GT-only-in-eval; every
repeat reported (no run dropped). Harsh metrics are already Q<=2 / GT-windowed.
"""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path

DATASETS = ["medium", "deep", "harsh"]
METHODS = {"bl": "baseline", "va4": "proposed"}
KEYS = [("rmse_h_m", "h(m)"), ("rmse_u_m", "u(m)"),
        ("yaw_rmse_deg", "yaw(deg)"), ("fixed_rate", "fix")]


def mean_std(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return None, None
    m = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs)) if len(xs) > 1 else 0.0
    return m, sd


def load(out: Path, rep: int, tag: str, ds: str):
    p = out / f"rep{rep}" / tag / ds / "metrics.json"
    if not p.is_file():
        return None
    return json.load(p.open())["metrics"]


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results/research/paper_final_20260720")
    report = {"out_root": str(out), "datasets": {}}
    print(f"\n=== PAPER FINAL MATRIX — {out} ===")
    for ds in DATASETS:
        runs = {tag: [load(out, r, tag, ds) for r in (1, 2, 3)] for tag in METHODS}
        n_done = {tag: sum(1 for m in runs[tag] if m) for tag in METHODS}
        report["datasets"][ds] = {"n_done": n_done, "metrics": {}, "paired": {}}
        print(f"\n## {ds.upper()}   (reps done: bl={n_done['bl']}/3 va4={n_done['va4']}/3)")
        header = "  metric      baseline(mean±std)      proposed(mean±std)     Δ(prop−base) mean   better"
        print(header)
        for key, lab in KEYS:
            bl_vals = [m[key] for m in runs["bl"] if m]
            va_vals = [m[key] for m in runs["va4"] if m]
            bm, bs = mean_std(bl_vals)
            vm, vs = mean_std(va_vals)
            # paired deltas where both reps present
            deltas = [va[key] - bl[key] for bl, va in zip(runs["bl"], runs["va4"])
                      if bl and va and bl[key] is not None and va[key] is not None]
            dm, ds_ = mean_std(deltas)
            # "better": lower is better for all these except fixed_rate (higher better)
            better_lower = key != "fixed_rate"
            nbet = sum(1 for d in deltas if (d < 0) == better_lower and d != 0)
            report["datasets"][ds]["metrics"][key] = {
                "baseline_mean": bm, "baseline_std": bs,
                "proposed_mean": vm, "proposed_std": vs,
            }
            report["datasets"][ds]["paired"][key] = {
                "deltas": deltas, "mean": dm, "std": ds_, "n_better": nbet, "n": len(deltas),
            }
            if bm is None or vm is None:
                print(f"  {lab:10s}  (incomplete)")
                continue
            arrow = "better" if (dm < 0) == better_lower else "worse"
            print(f"  {lab:10s}  {bm:8.3f} ± {bs:5.3f}      {vm:8.3f} ± {vs:5.3f}     "
                  f"{dm:+8.3f}          {nbet}/{len(deltas)} {arrow}")
    (out / "final_comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nWrote {out/'final_comparison.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
