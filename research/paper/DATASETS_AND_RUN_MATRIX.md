# Finalized dataset set + full run matrix (covariance/decision-layer study)

*2026-07-19. What data the study's numbers consume, what is on disk, what to download,
and the exact run matrix the master launcher (scripts/run_paper_all.sh) executes.*

## Dataset manifest (checked on disk 2026-07-19)

Root: `/media/theph/Data1/Research/dataset/`

| Dataset | Role in paper | Status | Size |
|---|---|---|---|
| **GICI-board 1.1** (open-sky) | Table 1: covariance equivalence vs ceres + cost; frozen-baseline guard | PRESENT ✓ | 1.4 G |
| **UrbanNav Deep** (Whampoa) | MAIN evaluation: solo n=3 all arms; 3-tier metrics | PRESENT ✓ | 71 G |
| **UrbanNav Medium** (TST) | Noise-floor calibration (0-fix parity) | PRESENT (symlinks → ~/Downloads) | — |
| GICI-board 3.1, 4.1 | Optional no-regression breadth (baseline preserved) | PRESENT ✓ | 1.7 G / 1.4 G |
| **UrbanNav Harsh** (Mongkok) | OPTIONAL 3rd urban tier; Table V ref RRR 6.73 m / 1.44° | **MISSING — download** | ~70 G |

### Conclusion on downloads
- **Core paper (everything currently drafted): NO new download needed.** 1.1 + Deep +
  Medium are all present and have produced today's numbers.
- **Two housekeeping items before a clean full run:**
  1. **Medium** currently lives as symlinks into `~/Downloads/UrbanNavDataset-master/`.
     For a self-contained, reproducible run, copy it fully onto `Data1` (so the paper
     run does not depend on `~/Downloads`). Functional as-is, but fragile.
  2. **Harsh (Mongkok)** — the ONLY genuine download, and only if we want the
     Medium→Deep→Harsh urban-severity sweep (strengthens external validity, directly
     comparable to Chi et al. Table V's third UrbanNav row). Source: UrbanNav-HK dataset
     (PolyU IPNL). Needs: rover RINEX, HKKT/base RINEX, broadcast eph, DCB, Xsens IMU,
     ZED2 left images. Then a convert pass into gici file-mode streams + a new
     `DATASETS["harsh"]` entry in `scripts/run_urbannav_rrr_baseline.py`
     (gt=UrbanNav_mongkok_raw.txt-equivalent, correct gps-week-day offset).

## Full run matrix (what scripts/run_paper_all.sh executes)

Solo protocol: ONE gici_main at a time, machine otherwise idle (post-file replay is
backpressure-paced + wall-clock solver budget ⇒ load-sensitive). Resumable: a run whose
`output/solution.txt` is already complete is skipped.

### Stage A — build + integrity gate
- `scripts/build_research.sh` (Release).
- Frozen-baseline guard: `run_baseline_1_1.sh` → author APE must stay
  0.029198 m / 0.470557° ± 0.005 / 0.1 (proves opt-in flags don't perturb baseline).

### Stage B — Table 1 (covariance equivalence + cost), GICI-board 1.1
- `run_va_1_1.sh` with the VA benchmark config → parse `[vaar-fast]` / `[vaar-benchmark]`
  lines from `run.stderr`: usable %, rel-err distribution, conservative-disagreement
  count, fast_ms vs ceres_ms. (Expected: 99.94% usable, 90.1% ≤1e-3, no material
  overconfidence — 171/177 conservative, 6 numerical ties ≤7.3e-4 — 28 ms vs 812 ms.)

### Stage C — UrbanNav Deep, solo (main results, Tables 3/6/7)
| Arm | Runner | n |
|---|---|---|
| Baseline | run_urbannav_rrr_baseline.py deep | 3 |
| VA-v4 (dose, recommended) | run_urbannav_rrr_va4.py deep | 3 |
| RF-Cauchy (soft float) | run_urbannav_rrr_rfcva.py deep | 3 |
| VA-v2 (gates) — context | run_urbannav_rrr_va.py deep | 1 |
| VA-v3 (expiry) — context | run_urbannav_rrr_va3.py deep | 1 |
| RF-Tukey (hard float) — context | run_urbannav_rrr_rfva.py deep | 1 (expected crash — recorded) |

### Stage D — UrbanNav Medium (noise floor)
- Baseline + VA-v4, n=1 each (both fix 0% ⇒ parity by construction; calibrates ~0.2 m floor).

### Stage E — evaluation (scripts/eval_paper_tables.py)
- Raw ENU RMSE (primary) + per-epoch p95/max for every run/arm → Tables 3, 6.
- 3-tier crosswalk (raw / ATE SE(3) / APE Sim(3)) via `eval_deep_3tier.sh` → Table 7
  (baseline Sim(3) should bracket Chi Table V's 2.46 m).
- Table 1 from Stage B logs.
- Emits `<out-root>/paper_tables.{json,md}`.

## Budget (Deep dominates)
- Deep run ≈ 15 min each × (3+3+3+1+1+1 = 12) = ~3 h. + Medium (~2×8 min) + 1.1 (~2×13 min)
  + eval (~10 min). **Full clean regeneration ≈ 3.5–4 h wall, unattended.**
- Disk: results ≈ a few GB (solutions are text). Harsh download (if chosen) ≈ 70 G.

## If adding Harsh
Add `DATASETS["harsh"]`, convert streams, then Stage C/E gain a `harsh` column
(BL/VA-v4/RF-Cauchy n=3). Same protocol, ~+1.5 h.
