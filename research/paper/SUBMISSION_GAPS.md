# Paper-1 clean re-run bible (cov-centric)

**Status:** experiments must be regenerated from scratch with the rebuilt binary.
Do **not** cite `results/research/paper_repro/` or old `logs/research/gici_board_va/1_1/`.

**3-turn sequential runbook (prepare → verify → stop):**  
`research/paper/RUNBOOK_3_TURNS.md`  
Fresh tree: `results/research/paper_cov_LATEST` → `*_3turns/`  
Prepare only: `scripts/prepare_paper_cov_3turns.sh` (does **not** start runs).

## DO NOT use

| Forbidden | Why |
|-----------|-----|
| `scripts/run_paper_all.sh` | Regenerates **VA-v4 / RF-Cauchy** old chain |
| `results/research/paper_repro/**` | Old numbers / old claims |
| `logs/research/gici_board_va/1_1/run.stderr` | 1787 `[vaar-fast]` lines but **no** `max_diag_rel` (pre-matrix binary) |
| `research/config/rtk_imu_camera_rrr_va_urbannav.yaml` | bootstrap 0.999 + threads=4 — **not** confirmatory |
| `research/config/rtk_imu_camera_rrr_va4_*.yaml` | Out of scope for this letter |

## Environment trap (fixed in runners)

Shell may have stale:
`URBANNAV_DATA_ROOT=.../UrbanNavDataset` (directory **missing**).
Real data lives at `/media/theph/Data1/Research/dataset/UrbanNav-HK-*`.
Runners now ignore a non-existent env root; launcher also exports the good path.
Still safest:
```bash
export URBANNAV_DATA_ROOT=/media/theph/Data1/Research/dataset
unset URBANNAV_DATA_ROOT   # also OK — auto-detect works
```

## Binary (must match)

```bash
ls -la build/gici_main          # rebuilt 2026-07-19 with matrix fields
# strings check:
rg -l 'max_diag_rel' src/fusion/rtk_imu_camera_rrr_va_estimator.cpp
```

Old board log deliberately rejected by:
`python3 scripts/eval_cov_matrix_metrics.py logs/research/gici_board_va/1_1/run.stderr` → `REJECT`.

## One-command clean campaign

```bash
# Machine idle. Solo gici_main. Fresh dated tree.
scripts/run_paper_cov_all.sh
# or:
scripts/run_paper_cov_all.sh results/research/paper_cov_manual
```

Stages:

| Stage | What | Pass criterion |
|-------|------|----------------|
| A | build + frozen baseline 1.1 guard | APE guard PASS |
| B | Board VA-1.1 fast vs Ceres | log has `max_diag_rel`; write `matrix_metrics.txt` |
| C | Medium iso shadow vs proposed | `PASS_TRAJ_IDENTITY` or **ABORT** |
| D | Deep n=1: shadow / proposed / local / ceres | complete solutions (~60k lines) |
| E | `deep_summary.txt` | numbers for LaTeX |

Ceres Deep timeout override: `--timeout-s 21600` (6 h).

## Manual equivalents (if not using the launcher)

```bash
# B — board matrix (NEW log path!)
GICI_VA_OUT=results/research/paper_cov_XXX/board_1_1/va \
GICI_VA_LOG=results/research/paper_cov_XXX/board_1_1/va \
  bash scripts/run_va_1_1.sh
python3 scripts/eval_cov_matrix_metrics.py results/research/paper_cov_XXX/board_1_1/va/run.stderr

# C — Medium iso
python3 scripts/run_urbannav_rrr_bl_iso.py medium --out-root results/research/paper_cov_XXX/iso/bl
python3 scripts/run_urbannav_rrr_va_iso.py medium --out-root results/research/paper_cov_XXX/iso/va
python3 scripts/check_float_path_identity.py \
  --a results/research/paper_cov_XXX/iso/bl/medium/output/solution.txt \
  --b results/research/paper_cov_XXX/iso/va/medium/output/solution.txt \
  --dataset medium
# Need: PASS_FIX_ZERO and PASS_TRAJ_IDENTITY (median Δh < 1 cm, p95 < 5 cm)

# D — Deep confirmatory (iso solver: threads=1, no wall-clock cap, bootstrap=0)
python3 scripts/run_urbannav_rrr_bl_iso.py      deep --out-root .../deep/shadow
python3 scripts/run_urbannav_rrr_va_iso.py      deep --out-root .../deep/proposed
python3 scripts/run_urbannav_rrr_va_local.py    deep --out-root .../deep/local
python3 scripts/run_urbannav_rrr_ceres_oracle.py deep --out-root .../deep/ceres --timeout-s 21600
```

## Config matrix (confirmatory)

| Arm | Template | Key flags |
|-----|----------|-----------|
| Shadow | `rtk_imu_camera_rrr_urbannav_iso.yaml` | baseline; threads=1; max_solver_time=1e9; bootstrap=0 |
| Proposed | `rtk_imu_camera_rrr_va_urbannav_iso.yaml` | fast=true; exact=false; bootstrap=0; joint_cost=false |
| Ceres oracle | `rtk_imu_camera_rrr_va_urbannav_ceres_oracle.yaml` | fast=false; exact=true |
| Local (neg.) | `rtk_imu_camera_rrr_va_urbannav_local.yaml` | fast=false; exact=false; delta=true |
| Board bench | `rtk_imu_camera_rrr_va_1_1.yaml` | fast=true; **benchmark=true** |

## Solution paths (easy to get wrong)

```text
$OUT/iso/bl/medium/output/solution.txt
$OUT/iso/va/medium/output/solution.txt
$OUT/deep/shadow/deep/output/solution.txt
$OUT/deep/proposed/deep/output/solution.txt
$OUT/board_1_1/va/run.stderr          # matrix log
$OUT/board_1_1/va/output/solution.txt
```

## What the paper needs from a successful run

1. **T1 matrix table:** Frobenius + max_diag_rel + psd_diff rate + gen_eig_max + runtime percentiles (`matrix_metrics.txt`)
2. **Float-path:** identity PASS on Medium (or a documented root-cause fix if FAIL)
3. **Ablation Deep:** shadow / local / proposed / ceres — same protocol, n=1 sequence first
4. **Headline claim:** proposed ≈ Ceres AR decisions/trajectory, much faster — not “beats shadow horizontal”

## If Stage C fails

Do **not** proceed to Deep claims. Debug:

1. Grep run logs for WlFix / AmbiguityError (NlFix rate alone is insufficient)
2. Confirm both arms used iso templates (bootstrap 0, threads 1)
3. Solo idle machine
4. Only then consider Evaluate/IMU side effects

## Wall-time estimate (idle machine)

| Stage | Approx |
|-------|--------|
| A guard | ~15–25 min |
| B board + Ceres compare each epoch | ~45–90 min |
| C Medium ×2 | ~40–80 min |
| D Deep shadow/proposed/local | ~45–90 min each |
| D Deep ceres | **2–6 h** |
| **Total** | **~8–14 h** |

## After runs

1. Update LaTeX tables from `$OUT` only  
2. Recompile PDF  
3. Update `CLAIM_AUDIT.md`  
4. Still no commit unless asked  
