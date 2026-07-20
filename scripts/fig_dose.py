#!/usr/bin/env python3
"""F5: histogram of P_s / std_cycles from [softw] acceptance logs (VA-v4 Deep)."""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
DEFAULT_LOGS = [
    REPO / f"results/research/urbannav_va4solo_20260718/run{k}/deep/run.log"
    for k in (1, 2, 3)
]
OUT = REPO / "research/paper/fig/F5_dose.pdf"
RE = re.compile(r"\[softw\]\s+n=(\d+)\s+Ps=([0-9.eE+-]+)\s+std_cycles=([0-9.eE+-]+)")


def load(logs: list[Path]) -> tuple[np.ndarray, np.ndarray]:
    ps, std = [], []
    for log in logs:
        if not log.is_file():
            continue
        for line in log.open(errors="ignore"):
            m = RE.search(line)
            if not m:
                continue
            ps.append(float(m.group(2)))
            std.append(float(m.group(3)))
    return np.asarray(ps), np.asarray(std)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", nargs="*", type=Path, default=DEFAULT_LOGS)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    ps, std = load(args.logs)
    if len(ps) == 0:
        raise SystemExit("no [softw] lines found")
    soft = std > 0.005
    args.out.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(5.4, 2.6))
    axes[0].hist(ps, bins=30, color="#1f4e79", alpha=0.85)
    axes[0].set_xlabel(r"$P_s$")
    axes[0].set_ylabel("Count")
    axes[0].set_title(f"Accepted subsets (n={len(ps)})")
    axes[1].hist(np.log10(std), bins=30, color="#b00020", alpha=0.85)
    axes[1].axvline(np.log10(0.005), color="#333", ls="--", lw=0.8)
    axes[1].set_xlabel(r"$\log_{10}$(std cycles)")
    axes[1].set_title(f"soft std>0.005: {int(soft.sum())}/{len(std)}")
    fig.suptitle("VA-v4 decision-confidence dose", fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out)
    print(f"wrote {args.out}  n={len(ps)} soft={int(soft.sum())} max_std={std.max():.3f}")


if __name__ == "__main__":
    main()
