# Decisive pre-submission validation (2026-07-20)

Two tests that were flagged as turning the paper from "maybe" to "certain". Run today.
All numbers trace to the referenced logs/scripts. No result invented.

---

## Task 1 — Covariance consistency (fast Q_aa vs exact Ceres): **PASS (positive)**

**Data:** existing board-1.1 log `results/research/nees/va_1_1/log/run.stderr` (1786
`[vaar-fast]` epochs, rebuilt matrix binary). No new run needed.
**Script:** `scripts/eval_qaa_consistency.py` (companion to `eval_cov_matrix_metrics.py`;
conditions on non-degenerate Ceres to remove the divide-by-~0 artifact).

The RAW relative-Frobenius tail (`eval_cov_matrix_metrics.py`: p95=1.5e6, max=1.2e8,
Loewner 8.6%) is a **metric artifact**: on the tail epochs the EXACT Ceres covariance
collapses toward singular (tr_ceres ~ 1e-6, right after a fix pins ambiguities), so the
relative error explodes purely from a vanishing denominator — there tr_fast is LARGER,
i.e. the fast estimate is the conservative one. Confirmed: the 10 worst epochs all have
tr_ceres ~ 1e-6, tr_fast ~ 1e3, ceres_ok=1, psd=1.

**Conditioned on non-degenerate Ceres (tr_ceres >= 1e-2, 92.5% of ok epochs):**

| metric | value |
|---|---|
| fast Q_aa vs exact Ceres, rel-Frobenius <= 1e-3 | **99.6%** |
| worst-direction non-conservative deviation (worst epoch) | **-1.17%** of per-ambiguity variance (p5 = -0.42%) |
| fast runtime p50 / p99 / max | **14.9 / 27.6 / 33.1 ms**, 0% over 50 ms |
| Ceres runtime p50 | 218.8 ms |
| median speedup | **~15x** |

**Honest wording (locked):** do NOT write "never overconfident" — exact Loewner
Qf>=Qc holds only ~1-9% by finite precision. Correct: *"matches the exact marginal
covariance to <=1e-3 on 99.6% of (non-degenerate) epochs; where non-conservative, the
worst-direction deficit is <=1.2% of the per-ambiguity variance; ~15x faster with no
epoch over the 50 ms budget."* This is frame-independent (no GT / no alignment) and
supports the "consistent + real-time" headline.

---

## Task 2 — Medium float-path identity (0-fix isolation): **FAIL_TRAJ_IDENTITY**

**Arms (both deterministic, threads=1, max_solver_time=1e9, same binary
build 2026-07-19_16:59):**
- BL-iso: `results/research/va_bakeoff_iso_20260720/bl/medium` (h=2.337, fix=0.00%)
- VA-iso: `results/research/iso_medium/va/medium` (h=2.304, fix=0.00%) — run 2026-07-20 13:40

**Check:** `scripts/check_float_path_identity.py` (medium).
- `PASS_FIX_ZERO` — both arms 100% quality-5, zero fixes of any kind (no NlFix, no WL).
- `FAIL_TRAJ_IDENTITY` — arm-to-arm Δh median **0.251 m**, p95 0.88 m, max 3.40 m
  (threshold: median < 1 cm, p95 < 5 cm).

**Root cause (diagnosed, not a test artifact):**
- First 50 epochs Δh = **0.000 m** — arms start byte-identical (rules out frame offset
  / stale-bl / config-base mismatch).
- Divergence then activates and stays bounded in a ~0.2-0.3 m band (Q1..Q4 medians
  0.185 / 0.330 / 0.268 / 0.200 m) — not runaway.
- Config diff between arms is ONLY: estimator class + VA covariance flags. Nothing else.
- Precise root cause (traced): `Graph::getMarginalAmbiguityCovariance` evaluates every
  active residual to assemble the information matrix; `ImuError::EvaluateWithMinimalJacobians`
  **relinearizes its preintegration in place** (`redoPreintegration`, imu_error.cpp:601)
  when the current gyro-bias has drifted past a threshold, permanently moving the factor's
  linearization point + `square_root_information_` (all `mutable` members). That moves the
  graph linearization → the next `optimize()` starts differently → the ~0.25 m float drift.
  Early epochs match exactly because the state still sits at the last relinearization point
  (redo does not fire); the divergence turns on once bias drift accumulates.

  CORRECTION to an earlier note: the `marginalization_error.cpp:1043` FATAL that crashed
  BL/harsh seed-1 is in the BASELINE estimator (`rtk_imu_camera_rrr`), which does NOT call
  the VA covariance path. It is a SEPARATE, pre-existing baseline issue on Harsh, NOT the
  same root cause as this float-path side effect. Do not conflate them.

**Conclusion:** turning on the VA covariance machinery **perturbs the float state
estimate** (~0.2-0.3 m), even with zero fixes. The covariance query is **not a
side-effect-free observer** — it writes back into the optimization graph state.

**Paper implication:**
- The headline (consistent + real-time covariance; Task 1) is UNAFFECTED and solid.
- The SECONDARY sub-claim "proposed changes only the AR decision; trajectory otherwise
  identical to baseline" must be **DROPPED**. VA is a full estimator variant, not a
  clean overlay. End-to-end accuracy (multi-seed table) already reflects this and is the
  correct place to compare.
- Reliability caveat to disclose: the covariance path has an estimator side effect and
  a reproduced crash on Harsh. This is a robustness limitation, not a headline killer.

**MAJOR CORRECTION (2026-07-20, after the fix): the FAIL was NOT a VA side effect.**
The ISO config (threads=1, max_solver_time=1e9) is **NOT bit-reproducible**. Two identical
BASELINE runs (same code, same binary, same config) differ by Δh median **0.29 m**, p95
1.83 m, max 3.14 m — essentially the same magnitude as the arm-to-arm VA-vs-BL difference
(0.25–0.35 m). Evidence: `results/research/iso_medium_fixed/bl` vs
`results/research/iso_determinism_check/bl_b`.

Consequences:
- The `FAIL_TRAJ_IDENTITY` result is dominated by **run-to-run nondeterminism**, not a
  VA-specific side effect. The earlier "VA perturbs the float path" conclusion is RETRACTED.
- The "first 50 epochs identical" observation is consistent with nondeterminism whose onset
  is delayed until the window/marginalization grows (unordered-container ordering + the
  deep/medium-urban ill-conditioning the paper is itself about amplify tiny roundoff).
- The IMU-relinearization side effect found by the code trace is REAL, but its effect is
  below the nondeterminism floor, so the trajectory-identity test can neither confirm nor
  refute it. The applied fix is a correctness/hygiene improvement, NOT a fix for a measured
  symptom.
- The project's "ISO = reproducible n=1" premise (CURRENT_STATE) is FALSE. Deterministic
  n=1 confirmatory claims must be replaced by distributional (multi-seed mean±std) reporting,
  OR determinism must be engineered (order the containers feeding ceres — a larger, riskier
  frozen-code change).
- Correct path: DROP the trajectory-identity sub-claim entirely (it is not measurable in a
  nondeterministic, ill-conditioned system); present VA as a full estimator variant and
  compare end-to-end via multi-seed mean±std.

---

**Fix applied 2026-07-20 (user-approved), now understood as code-hygiene only:**
Added `ErrorInterface::setSuppressRelinearization(bool)` (no-op default), overridden in
`ImuError` to gate the in-place `redoPreintegration` (imu_error.cpp:601 now
`if (redo_ && !suppress_relinearization_)`, leaving `redo_` latched so the next real
optimize() relinearizes normally). The three covariance/information observers set the flag
around their evaluations and restore it: `getMarginalAmbiguityCovariance`,
`getLocalCrossInformation`, and `computeCovariance` (RAII guard around
`ceres::Covariance::Compute`). Files: `include/gici/estimate/error_interface.h`,
`include/gici/imu/imu_error.h`, `src/imu/imu_error.cpp`, `src/estimate/graph.cpp`.
Binary rebuilt 2026-07-20 13:58 (rc=0). Baseline behaviour byte-identical (baseline never
sets the flag). Validation (fixed-binary bl-iso vs va-iso Medium float-path): PENDING.

Note: the BASELINE Harsh crash (`marginalization_error.cpp:1043`) is a separate issue and
is NOT addressed by this fix.

---

## Task 3 (G3) — Baseline Harsh crash: ROOT-CAUSED + FIXED (original GICI bug)

**Correcting the attribution:** the crash is NOT the authors' algorithm being broken (they
report Harsh RRR = 6.73 m), NOT our re-prepped data (Harsh runs clean at threads=1,
h=7.569 m), and NOT our code change (our only edit to marginalization_error.* is a purely
additive read-only accessor, off the baseline path). It is a **latent ordering bug in the
original GICI marginalization flow**, exposed by deep-urban degraded conditions and made
more frequent by our threads=4 real-time config (timing shifts which epochs hit it — it is
a data-dependent branch, NOT a data race).

**Root cause** (`src/estimate/estimator_base.cpp:93`, `applyMarginalization`): residual
blocks get added to the prior (each `addResidualBlock` sets `error_computation_valid_ =
false`, marginalization_error.cpp:151) on epochs that marginalize ZERO parameter blocks —
e.g. `addImuStateMarginBlock` early-returns on overlap states while sibling helpers still
add IMU/GNSS residuals; in harsh urban canyons cycle-slips / satellite / landmark loss
leave nothing to marginalize. The old guard `if (parameter_blocks_to_be_marginalized.size()
> 0) updateErrorComputation();` then SKIPS the recompute, `marginalizeOut(empty)`
early-returns without touching the flag (marginalization_error.cpp:556), and the prior is
re-added + solved with an invalid cached decomposition -> FATAL
`CHECK(error_computation_valid_)` at marginalization_error.cpp:1043. Intermittent,
~28883 epochs into BL/harsh only. Independently verified against the source.

**Fix** (`estimator_base.cpp:93`): recompute whenever the prior is non-empty, not only when
params were marginalized:
```cpp
if (marginalization_error_ && marginalization_error_->parameterBlocks() > 0) {
  marginalization_error_->updateErrorComputation();
}
```
`updateErrorComputation()` self-guards (no-op when already valid, marginalization_error.cpp:900),
so this is behavior-preserving for every previously-working epoch and only refreshes the
derived `J_/e0_/S_` from the same `H_/b0_` (no state/weight/linearization change). Guarding
on `parameterBlocks() > 0` (rather than an unconditional call) additionally avoids a
`maxCoeff`-on-empty UB in `updateErrorComputation` for the degenerate empty-prior case.
Rebuilt 2026-07-20 14:47 (rc=0). Validation (BL/harsh threads=4 past epoch 28883 without
the CHECK): **CONFIRMED 2026-07-20 15:20.**

**G3 crash-fix validation result** (`results/research/harsh_crashcheck_fixed/`,
rtk_imu_camera_rrr baseline, num_threads=4, both fixes in the final binary):
- Reached `final_gpgga = 32609` (past the old crash point ~epoch 28883); watchdog stopped it
  cleanly at EOF-stall (`exit_code=-2`, `watchdog_stopped=true` — the *normal* termination for
  every dataset run, incl. author_exact). **Zero `error_computation_valid_` / abort / OOM markers.**
- **The fix is a numeric no-op on the baseline.** Harsh rmse_h = 7.613 m now vs 7.569 m on the
  earlier pre-fix clean run (base5s_compare/harsh_iso_fix) = Δ0.044 m (0.6%, run-to-run noise);
  identical n_matched=12090, fixed_rate=0.0, float_rate=0.995. Crash removed WITHOUT moving the
  baseline number — the design goal.

**Two honest caveats (neither introduced by the fix):**
1. Harsh baseline rmse_h ≈ 7.6 m vs author's published APE 6.73 m (~13% reproduction gap;
   pre-fix run was already 7.57 m). Disclose as an environment/reproduction gap in the paper.
2. Harsh baseline fixed_rate = 0.0 (all-float deep-urban); VA variant reaches ~1.7%.

**Consistency after both fixes:** R1 on the VA-fixed binary still holds — fast Q_aa vs
exact Ceres 99.4% <=1e-3 (conditioned), worst-dir deficit <=2.0% per-ambiguity variance,
~16.3x faster, 0% over 50 ms. Headline "consistent + real-time" intact.

**Paper framing note:** this is publishable as a found-and-fixed reproducibility bug in the
baseline framework, disclosed honestly — NOT as an author defect.
