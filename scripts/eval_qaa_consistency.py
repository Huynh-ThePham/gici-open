#!/usr/bin/env python3
"""Honest Q_aa consistency table: fast marginal AR covariance vs exact Ceres.

Companion to eval_cov_matrix_metrics.py. That script reports RAW relative-Frobenius
percentiles, which are dominated by a tail of epochs where the EXACT Ceres covariance
collapses toward singular (tr_ceres ~ 1e-6, e.g. immediately after a fix pins the
ambiguities). On those epochs the relative error explodes purely because the
denominator -> 0, NOT because the fast covariance diverges (there tr_fast is LARGER,
i.e. the fast estimate is the conservative one). Reporting the raw tail therefore
understates consistency.

This script conditions on non-degenerate Ceres (tr_ceres >= --tc-floor) and reports:
  * relative-Frobenius agreement on the conditioned set,
  * Loewner rate (fraction with Qf >= Qc exactly), AND the magnitude of any
    non-conservative deviation, normalized by the mean per-ambiguity Ceres variance
    (tr_ceres / n) -- so "how overconfident, if at all" is quantified, not just a
    yes/no Loewner flag that finite precision almost never passes,
  * runtime percentiles + speedup.

Honesty rules (locked): do NOT claim "never overconfident" (Loewner rate is low by
exact arithmetic). DO report that where the fast covariance is non-conservative the
worst-direction deficit is a tiny fraction of the per-ambiguity variance. All numbers
trace to the input log.

Usage: python3 scripts/eval_qaa_consistency.py <run.stderr> [--tc-floor 1e-2]
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


def _f(s: str) -> float:
    try:
        return float(s)
    except (ValueError, TypeError):
        return float("nan")


def g(line: str, key: str) -> float:
    m = re.search(rf"{key}=([0-9.eE+-]+)", line)
    return _f(m.group(1)) if m else float("nan")


def pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    ys = sorted(xs)
    k = (len(ys) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return ys[lo] if lo == hi else ys[lo] * (hi - k) + ys[hi] * (k - lo)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--tc-floor", type=float, default=1e-2,
                    help="exclude epochs whose EXACT tr_ceres < this (degenerate/near-singular Ceres)")
    args = ap.parse_args()

    text = args.log.read_text(errors="ignore")
    if "max_diag_rel=" not in text:
        raise SystemExit(f"REJECT: {args.log} has no max_diag_rel (old pre-matrix log).")

    rows = []
    for l in text.splitlines():
        if "[vaar-fast]" not in l:
            continue
        rows.append(dict(
            ok=g(l, "fast_ok"), re=g(l, "rel_err"), n=g(l, "n"),
            tf=g(l, "tr_fast"), tc=g(l, "tr_ceres"),
            med=g(l, "min_eig_diff"), psd=g(l, "psd_diff"),
            fms=g(l, "fast_ms"), cms=g(l, "ceres_ms"),
        ))
    if not rows:
        raise SystemExit("no [vaar-fast] lines")

    N = len(rows)
    n_ok = sum(1 for r in rows if r["ok"] == 1)
    # timing over all epochs (validity of the covariance does not gate its cost)
    fms = [r["fms"] for r in rows if not math.isnan(r["fms"])]
    cms = [r["cms"] for r in rows if not math.isnan(r["cms"])]

    ok_rows = [r for r in rows if r["ok"] == 1]
    sub = [r for r in ok_rows if r["tc"] >= args.tc_floor]
    n_degen = len(ok_rows) - len(sub)

    re_ = [r["re"] for r in sub if not math.isnan(r["re"])]
    psd_scored = [r["psd"] for r in sub if r["psd"] in (0.0, 1.0)]
    # worst-direction non-conservative deviation, normalized by mean per-ambiguity ceres variance
    ovr = []
    for r in sub:
        scale = max(r["tc"] / max(r["n"], 1.0), 1e-12)
        ovr.append(r["med"] / scale)  # >=0 conservative; <0 fast smaller (overconfident) in worst dir

    print(f"log: {args.log}")
    print(f"epochs total={N}  fast_ok={n_ok} ({100*n_ok/N:.1f}%)")
    print(f"excluded degenerate-Ceres (tr_ceres<{args.tc_floor:g}): {n_degen}  "
          f"({100*n_degen/max(n_ok,1):.1f}% of ok)  -> conditioned n={len(sub)}")
    print()
    print("CONSISTENCY (fast Q_aa vs exact Ceres Q_aa, conditioned set):")
    print(f"  rel-Frobenius  p50={pct(re_,50):.2e}  p95={pct(re_,95):.2e}  max={max(re_):.2e}")
    print(f"  frac <=1e-3    {100*sum(x<=1e-3 for x in re_)/len(re_):.1f}%   "
          f"<=1e-2 {100*sum(x<=1e-2 for x in re_)/len(re_):.1f}%")
    print(f"  Loewner Qf>=Qc exact rate  {100*sum(psd_scored)/max(len(psd_scored),1):.1f}%  "
          f"(low by finite precision -- see deviation magnitude below)")
    print(f"  worst-dir deviation / (tr_ceres/n)  p5={pct(ovr,5):.4f}  p50={pct(ovr,50):.4f}  "
          f"min={min(ovr):.4f}")
    print(f"    (>=0 conservative; negative = fractional per-ambiguity variance deficit in worst dir)")
    print()
    print("RUNTIME (all epochs):")
    print(f"  fast_ms   p50={pct(fms,50):.1f}  p95={pct(fms,95):.1f}  p99={pct(fms,99):.1f}  max={max(fms):.1f}")
    print(f"  ceres_ms  p50={pct(cms,50):.1f}  p95={pct(cms,95):.1f}  max={max(cms):.1f}")
    print(f"  speedup (median ceres/fast)  {pct(cms,50)/pct(fms,50):.1f}x   >50ms fast: "
          f"{100*sum(x>50 for x in fms)/len(fms):.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
