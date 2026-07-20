#!/usr/bin/env python3
"""Aggregate the author-exact multi-seed campaign into a mean +/- std table.

Layout expected (produced by scripts/run_author_exact_multiseed.sh):
    <root>/{bl,va}/seed{N}/{scene}/metrics.json

Reports, per arm x scene, mean +/- sample-std over seeds for the metrics that
matter, alongside Chi et al. RA-L Table V references. Prints markdown to stdout
and writes <root>/multiseed_table.md + <root>/multiseed_table.json.

Honesty note: the config is non-deterministic; std is the whole point. Do NOT
claim BL-vs-VA differences smaller than the combined std.
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys

# Chi et al. RA-L UrbanNav RRR Table V references (pos APE m / rot deg).
PAPER = {
    "medium": {"pos_m": 3.40, "rot_deg": 1.30},
    "deep": {"pos_m": 2.46, "rot_deg": 1.64},
    "harsh": {"pos_m": 6.73, "rot_deg": 1.44},
}
SCENES = ["medium", "deep", "harsh"]
ARMS = ["bl", "va"]
ARM_LABEL = {"bl": "Baseline (reimpl)", "va": "VA (proposed)"}
# metric key -> (json field, scale, unit, fmt)
METRICS = [
    ("rmse_h_m", "rmse_h_m", 1.0, "m", "{:.3f}"),
    ("rmse_u_m", "rmse_u_m", 1.0, "m", "{:.3f}"),
    ("yaw_rmse_deg", "yaw_rmse_deg", 1.0, "deg", "{:.3f}"),
    ("fixed_pct", "fixed_rate", 100.0, "%", "{:.1f}"),
]


def mean_std(xs: list[float]) -> tuple[float, float, int]:
    n = len(xs)
    if n == 0:
        return (float("nan"), float("nan"), 0)
    m = sum(xs) / n
    if n == 1:
        return (m, 0.0, 1)
    var = sum((x - m) ** 2 for x in xs) / (n - 1)  # sample std
    return (m, math.sqrt(var), n)


def collect(root: str) -> dict:
    out: dict = {}
    for arm in ARMS:
        out[arm] = {}
        for scene in SCENES:
            paths = sorted(glob.glob(os.path.join(root, arm, "seed*", scene, "metrics.json")))
            seeds = []
            for p in paths:
                try:
                    with open(p) as f:
                        m = json.load(f)["metrics"]
                    seeds.append({"path": p, "metrics": m})
                except Exception as exc:  # noqa: BLE001
                    print(f"WARN cannot read {p}: {exc}", file=sys.stderr)
            out[arm][scene] = seeds
    return out


def render(root: str, data: dict) -> tuple[str, dict]:
    lines: list[str] = []
    js: dict = {"root": root, "paper_ref": PAPER, "arms": {}}
    lines.append("# Author-exact multi-seed table (mean +/- sample-std)\n")
    lines.append(
        "Config: relative_frequency=1.0, num_threads=4, max_solver_time=0.04 "
        "(author real-time, NON-deterministic). Framing: covariance method; "
        "accuracy = comparable/honest vs Chi et al. Table V, NOT an accuracy win.\n"
    )
    for scene in SCENES:
        ref = PAPER[scene]
        lines.append(f"\n## {scene}  (paper Table V: {ref['pos_m']:.2f} m / {ref['rot_deg']:.2f} deg)\n")
        header = "| arm | n | h RMSE (m) | U RMSE (m) | yaw RMSE (deg) | fixed (%) |"
        lines.append(header)
        lines.append("|---|---|---|---|---|---|")
        for arm in ARMS:
            seeds = data[arm][scene]
            js.setdefault("arms", {}).setdefault(arm, {})[scene] = {"n": len(seeds), "metrics": {}}
            cells = [ARM_LABEL[arm]]
            n_seen = len(seeds)
            cells.append(str(n_seen))
            for key, field, scale, unit, fmt in METRICS:
                xs = []
                for s in seeds:
                    v = s["metrics"].get(field)
                    if v is not None:
                        xs.append(v * scale)
                m, sd, n = mean_std(xs)
                js["arms"][arm][scene]["metrics"][key] = {"mean": m, "std": sd, "n": n}
                if n == 0:
                    cells.append("-")
                elif n == 1:
                    cells.append(f"{fmt.format(m)} (n=1)")
                else:
                    cells.append(f"{fmt.format(m)} +/- {fmt.format(sd)}")
            lines.append("| " + " | ".join(cells) + " |")
    lines.append(
        "\n---\nReading guide: a BL-vs-VA gap is only meaningful if it exceeds the "
        "combined std of the two arms. On horizontal (h RMSE) the method does not "
        "beat baseline; report VA's edge (if any) on U RMSE / yaw / fixed-rate only, "
        "and only where mean-gap > combined-std.\n"
    )
    return "\n".join(lines), js


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("usage: eval_author_exact_multiseed.py <root>", file=sys.stderr)
        return 2
    root = argv[1]
    data = collect(root)
    md, js = render(root, data)
    print(md)
    with open(os.path.join(root, "multiseed_table.md"), "w") as f:
        f.write(md + "\n")
    with open(os.path.join(root, "multiseed_table.json"), "w") as f:
        json.dump(js, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
