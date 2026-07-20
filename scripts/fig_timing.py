#!/usr/bin/env python3
"""F3: fast_ms vs ceres_ms vs graph size from [vaar-fast] log lines."""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[1]
DEFAULT_LOG = REPO / "logs/research/gici_board_va/1_1/run.stderr"
OUT = REPO / "research/paper/fig/F3_timing.pdf"

# Flexible: capture common fields if present
RE = re.compile(
    r"\[vaar-fast\].*?"
    r"\bfast_ms=(?P<fast>[0-9.]+).*?"
    r"\bceres_ms=(?P<ceres>[0-9.]+).*?"
    r"\bgraph_pb=(?P<pb>[0-9]+)",
)


def parse(log: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    fast, ceres, pb = [], [], []
    for line in log.open(errors="ignore"):
        if "[vaar-fast]" not in line:
            continue
        m = RE.search(line)
        if not m:
            continue
        fast.append(float(m.group("fast")))
        ceres.append(float(m.group("ceres")))
        pb.append(int(m.group("pb")))
    f = np.asarray(fast)
    c = np.asarray(ceres)
    p = np.asarray(pb) if pb else None
    if p is not None and np.all(p < 0):
        p = None
    return f, c, p


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=Path, default=DEFAULT_LOG)
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()
    fast, ceres, pb = parse(args.log)
    if len(fast) == 0:
        raise SystemExit(f"no [vaar-fast] lines in {args.log}")
    args.out.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(4.2, 3.0))
    if pb is not None:
        order = np.argsort(pb)
        ax.scatter(pb[order], ceres[order], s=8, alpha=0.35, c="#888888", label="ceres")
        ax.scatter(pb[order], fast[order], s=8, alpha=0.55, c="#1f4e79", label="fast")
        ax.set_xlabel("Graph parameter blocks")
    else:
        x = np.arange(len(fast))
        ax.scatter(x, ceres, s=6, alpha=0.35, c="#888888", label="ceres")
        ax.scatter(x, fast, s=6, alpha=0.55, c="#1f4e79", label="fast")
        ax.set_xlabel("AR epoch index")
    ax.set_ylabel("Time (ms)")
    ax.set_yscale("log")
    ax.legend(frameon=False, fontsize=8)
    ax.set_title(f"Ambiguity covariance cost (n={len(fast)})")
    fig.tight_layout()
    fig.savefig(args.out)
    print(f"wrote {args.out}  n={len(fast)}  fast_mean={fast.mean():.1f} ceres_mean={ceres.mean():.1f}")


if __name__ == "__main__":
    main()
