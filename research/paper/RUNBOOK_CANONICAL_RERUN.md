# Canonical re-run RUNBOOK (Paper 1) — 2026-07-20

Goal: rebuild the canonical result set from scratch, **each config run 3× to lock the
run-to-run variance (dao động → report mean ± std)**. GICI first (complete + check), then
UrbanNav.

## Why 3× / why solo
GICI + UrbanNav configs are **wall-clock-bounded** (`max_solver_time`) with `num_threads:4`
⇒ results are load-sensitive and **not bit-reproducible** (confirmed: run-to-run Δ up to a
few metres). Two rules follow, both baked into the runners:
- **SOLO**: one `gici_main` at a time on an otherwise-idle machine. Do NOT run GICI and
  UrbanNav (or two arms) at once — it biases the numbers.
- **n=3**: the 3 repetitions ARE the dao-động measurement. Every headline number is
  reported mean ± std over its 3 runs.

## Prerequisites (already done — verify)
- Clean P1 binary: `build/gici_main` (rebuilt 2026-07-20 from committed HEAD `dd40c7b`, no VIO).
- Dataset paths fixed: `scripts/dataset_paths.sh` → `GICI_DATA_ROOT=/media/theph/Data1/Research/dataset/Gici`.
- Data verified: all 12 GICI boards have `gnss_{rover,reference,ephemeris}.bin + imu.bin +
  camera.bin + ground_truth.txt`, non-empty; UrbanNav Deep/Medium/Harsh present with GT.
- Rebuild if unsure: `./scripts/build_research.sh`

---

# STAGE 1 — GICI (12 boards × 3, do this first)

Runner: `scripts/run_gici_canonical.sh` (validated: per-board dataset + correct RTCM
`start_time` substitution, resumable via `.done` markers, stall/ephemeris/max-wall watchdog).

Arms:
- `baseline` — frozen baseline estimator (`rtk_imu_camera_rrr`).
- `va` — VA fast marginal covariance, **benchmark OFF** (the method; fast).
- `va_bench` — VA **+ per-epoch fast-vs-ceres equivalence logging** (benchmark ON; ~10× slower).

### 1A. À LA CARTE — one board at a time (run whenever you're free)
Every command writes into the SAME out-root `results/research/gici_canonical/`, so the
result set **accumulates** across days. Each is independent + resumable (finished runs skip
via `.done`). Run board 1.1 today, 1.2 tomorrow, etc. — order does not matter.

Pattern (arg 4 = the board):
```bash
cd /home/theph/ws_ncs/gici_vision_aided_ar
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "<BOARD>"
```
Copy-paste, one board per session (each does baseline + VA, 3 runs = 6 gici_main; small
boards ~1–2 h, **5.1/5.2 much longer**):
```bash
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "1.1"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "1.2"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "2.1"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "2.2"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "3.1"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "3.2"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "3.3"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "4.1"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "4.2"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "4.3"
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "5.1"   # large
scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" "5.2"   # large
```
Run in the background if you want to close the terminal:
`nohup <the command> >> results/research/gici_canonical.nohup 2>&1 &`
Output: `results/research/gici_canonical/<board>/{baseline,va}/run{1,2,3}/`.

**Alternative — all 12 in one go** (unattended, still solo internally):
```bash
nohup scripts/run_gici_canonical.sh results/research/gici_canonical 3 "baseline va" \
   > results/research/gici_canonical.nohup 2>&1 &
```

### 1B. VA equivalence benchmark (fast Q_aa vs exact ceres) — pick ONE scope
This produces Table 1 (the consistency headline). It is slow (ceres every AR epoch).

- **Board 1.1 only** (paper default — open-sky proof), n=3:
  ```bash
  nohup scripts/run_gici_canonical.sh \
        results/research/gici_canonical 3 "va_bench" "1.1" \
     >> results/research/gici_canonical.nohup 2>&1 &
  ```
- **All 12 boards** (stronger: consistency across every scenario, but MUCH longer —
  boards 5.1/5.2 are ~8 G / 250k-epoch, expect many hours each):
  ```bash
  nohup scripts/run_gici_canonical.sh \
        results/research/gici_canonical 3 "va_bench" \
     >> results/research/gici_canonical.nohup 2>&1 &
  ```

### Monitor / resume
```bash
tail -f results/research/gici_canonical/run_gici_canonical.log     # progress (RUN/DONE/WARN)
pgrep -x gici_main && echo running || echo idle                    # solo check
grep -c "^\[" results/research/gici_canonical/run_gici_canonical.log
```
Relaunch the exact same command to resume after any interruption.

### 1C. CHECK GICI before moving on
Consistency (per VA-bench run → aggregate mean±std):
```bash
for f in results/research/gici_canonical/*/va_bench/run*/log/run.stderr; do
  echo "== $f =="; python3 scripts/eval_qaa_consistency.py "$f"
done
```
Accuracy + frozen-baseline guard use the **author eval pipeline** (the trusted path that
locked the 0.029198 m / 0.470557° baseline — GICI GT is Inertial-Explorer format, evaluated
via `ie_to_nmea` → align → `evo_ape`, NOT the UrbanNav ENU eval):
```bash
./scripts/build_eval_tools.sh          # once: builds tools/evaluation/{format_converters,alignment}
scripts/run_author_eval_1_1.sh         # board 1.1 baseline guard vs research/baseline/expected_1_1.json
```
Per-board × arm × 3-run accuracy mean±std for all 12 boards is the one eval piece still to
generalize (a thin wrapper over the same author tools, aggregating the 3 runs). Finalize it
while Stage 1 runs — the runs are the long pole, not the eval.

**Do NOT start Stage 2 until Stage 1 numbers look right** (consistency ~99% ≤1e-3 on
non-degenerate epochs; baseline guard within tolerance).

---

# STAGE 2 — UrbanNav (only after Stage 1 checks pass)

Runners (Python; each takes `<dataset> --out-root <dir>`; dataset ∈ deep/medium/harsh):
`run_urbannav_rrr_baseline.py`, `run_urbannav_rrr_va4.py` (VA-v4 dose, main),
`run_urbannav_rrr_rfcva.py` (RF-Cauchy boundary), `run_urbannav_rrr_rfva.py`
(RF-Tukey boundary, expected crash — recorded), `run_urbannav_rrr_va.py` (VA-v2 context).

### 2A. À LA CARTE — one dataset (× its arms × 3 runs) per session
Same idea: run `medium` one day, `deep` another, `harsh` another. All accumulate under
`results/research/urbannav_canonical/`. SOLO (one at a time). Each line below does the 3
runs for one (dataset, arm); resume-safe (skips a run whose solution.txt exists).

Runner map: `baseline`→`run_urbannav_rrr_baseline.py`, `va4`→`run_urbannav_rrr_va4.py`,
`rfcva`→`run_urbannav_rrr_rfcva.py` (add `va`/`rfva` for context arms if wanted).

```bash
cd /home/theph/ws_ncs/gici_vision_aided_ar
# --- pick ONE dataset (ds) and run its arms, 3x each ---
ds=medium        # or: deep | harsh
for arm_py in baseline:baseline va4:va4 rfcva:rfcva; do
  arm="${arm_py%%:*}"; py="run_urbannav_rrr_${arm_py##*:}.py"
  for k in 1 2 3; do
    out="results/research/urbannav_canonical/$ds/$arm/run$k"
    [ -f "$out/$ds/output/solution.txt" ] && { echo "skip $ds $arm run$k"; continue; }
    python3 "scripts/$py" "$ds" --out-root "$out"
  done
done
```
Change `ds=` and rerun next day. Harsh auto-applies its GT quality gate (`gt_q_max=2`) +
eval window inside the runner.

### 2B. Evaluate + assemble tables (mean±std)
```bash
python3 scripts/eval_paper_tables.py --root results/research/urbannav_canonical
# emits paper_tables.{json,md} with per-arm mean(μ) over the 3 runs.
```

---

## Notes
- Nondeterminism is a legitimate real-time behavior (queue-depth sparsify + wall-clock
  solver budget), NOT a bug — so mean±std over n=3 is the correct, honest reporting, in
  line with real-time GNSS SOTA. (Determinism-pinned `_det` config exists only as an
  internal corroboration, not the reporting path.)
- GICI accuracy uses the AUTHOR eval pipeline (IE→NMEA→align→evo_ape), not a custom parser —
  GICI solution.txt is NMEA but GT is Inertial-Explorer format; the author tools are the
  trusted, already-validated path. Generalizing them to 12 boards + n=3 mean±std is the
  remaining finalize-while-running task.
- Disk: text solutions are small; boards 5.1/5.2 solutions are the largest. Results live
  under `results/research/` (git-ignored).
