# Bounded GICI audit — adversarially verified findings (2026-07-20)

Scope: only the code paths our experiments execute (marginalization/error-computation,
covariance query, ambiguity resolution). Each auditor self-refuted; every item below was then
re-verified by reading the source directly. Detail files: `marginalization.md`, `covariance.md`,
`ambiguity.md`.

Bottom line: **no bug found that fabricates or hides the vision effect.** One CONFIRMED
methodological confound that is paper-critical (needs an ablation run, not a code fix), one
wording/scoping fix for the headline, and three benign/latent items.

---

## 1. CONFIRMED — Vision-AR fix-rate comparison is confounded (paper-critical; NOT a code bug)

**What the experiments actually do** (verified in code + all configs):
- Baseline `rtk_imu_camera_rrr_urbannav.yaml`: AR covariance = coarse **GNSS-only shadow**
  `RtkEstimator` (`rtk_imu_camera_rrr_estimator.cpp:799-822`, `estimateAmbiguityCovariance`,
  self-described "parallel optimizer … forcely set some empirical constraints … coarse covariance").
- VA `*_va_*` configs (all set `ar_use_fast_marginal_covariance: true`,
  `ar_use_exact_joint_covariance: false`, `ar_use_delta_information: false`): AR covariance =
  **full joint-graph marginal** `Graph::getMarginalAmbiguityCovariance`
  (`rtk_imu_camera_rrr_va_estimator.cpp:83`), which includes **GNSS + IMU + vision**.
- The fast-marginal call takes **no ablation argument** (`graph.cpp:307`), so in the configs we
  actually run there is **no way to turn vision off within this path**. `ablate_reprojection_in_ar`
  only affects the delta-information path (`va_estimator.cpp:298`), which is disabled.

**Consequence:** "VA improves fix rate over baseline" conflates two changes:
(1) coarse GNSS-only shadow → full joint-graph marginal (adds IMU + proper marginalization even
with zero vision), and (2) adding vision. It is **not** a clean vision-only attribution. This
directly touches the paper's "fix win" sub-claim.

**Good news:** both the consistency headline AND the fix-rate gain run through the *same*
ceres-validated `getMarginalAmbiguityCovariance` — so the gain is NOT driven by the
"experimental/overconfident" delta path (that path is off). The result is real; only the
*attribution* needs an ablation.

**Decisive control to close it:** run the VA estimator with the **same full-marginal path but
reprojection residuals excluded** (vision off, IMU+GNSS full-graph on), vs vision on. The delta is
the pure vision contribution. Options: (a) thread a `drop_reprojection` flag into
`getMarginalAmbiguityCovariance` (mirrors the existing `getLocalCrossInformation` support) and add a
`va_noreproj` config; (b) a VA variant with camera reprojection disabled in the graph. Either yields
an apples-to-apples vision-on/off comparison on the identical covariance machinery.

## 2. CONFIRMED (wording) — "exact Ceres covariance" is at the suppressed IMU linearization

- `imu_error.cpp:600-605`: `suppress_relinearization_` skips `redoPreintegration()` for **both** the
  fast path and the ceres reference. They agree with each other (reported `rel_err` is valid), but
  the reference is **not** stock `ceres::Covariance` on a freshly re-preintegrated graph.
- Fix is **paper wording only** (no code change): describe the comparison as "at the optimizer's own
  linearization, with IMU preintegration held fixed," rather than an unqualified "exact Ceres
  covariance." Low severity.

## 3. CONFIRMED (benign, upstream) — partial-AR "Variance" DelMethod sorts backwards

- `ambiguity_resolution.cpp:530-533`: `Variance` sorts **descending** by std → reduction keeps the
  worst-variance ambiguities and drops the best (opposite of `Elevation`/`Fractional`). Upstream
  (commit `c785e27`, unmodified). **Impact-neutral:** a higher-variance subset has a lower LAMBDA
  ratio → harder to pass, so it produces fewer/no fixes (never false ones), symmetric across baseline
  and VA; any freak fix is caught by the post-fix range-cost check. Optional one-line fix
  (`lhs.std < rhs.std`) only if we want to improve AR efficiency — requires a before/after fix-rate
  regression on all datasets first.

## 4. SUSPECTED (low, latent) — `solveAmbiguityLambda` ignores `lambda()` status

- `ambiguity_common.cpp:356-382`: `lambda()` return unchecked, `residuals` read possibly
  uninitialized on LD failure → garbage ratio. Backstopped by PD gates + range-cost check, so not
  result-affecting today. Trivial defensive fix (capture status; on non-zero set ratio=0/return
  false; zero-init `residuals`).

## 5. SUSPECTED (very low, benign) — all-fixed-prior empty-matrix eigen-solve

- `estimator_base.cpp` marginalization guard counts fixed zero-minimal-dim blocks; an all-fixed
  prior gives `parameterBlocks()>0` with `H_.cols()==0`. Practically unreachable under NDEBUG. A
  `residualDim()>0` guard closes it fully. The applied ordering fix itself is verified correct.

---

## Not bugs (checked and dismissed)
Ordering/permutation of fast vs ceres blocks; information-vs-covariance; Schur sign/equilibration
scale; landmark-strip off-by-one; prior double-count/omission; stale factorization; PSD
false-validation; loss-correction mismatch; cycle-slip reset; reference-sat/DD bookkeeping
(`findMatch`/`addStableFixationToGraph` are dead code); float→fixed constraint units; success-rate
gates (conservative-only); thread-safety in all three subsystems. See detail files for the concrete
trigger + refutation of each.

---

## Addendum (2026-07-20): deterministic vision ablation — vision HURTS AR on Deep

Ran the `exclude_reprojection` ablation (ON vs OFF differ ONLY in the AR covariance; float solution
identical) deterministically (num_threads=1) on UrbanNav Deep:

| | rmse_h | rmse_u | yaw | fixed_rate |
|---|---|---|---|---|
| VISION_ON  | 3.617 | 2.825 | 0.679 | 2.2% (340) |
| VISION_OFF | 3.437 | 1.071 | 0.705 | 4.0% (610) |

Vision-in-covariance is net NEGATIVE: vertical >2x worse (1.07->2.83 m), fewer fixes (610->340),
slightly worse horizontal, negligible yaw gain. threads=4 gave a bit-identical fix split (510/510)
— AR outcome is numerically sensitive to threading, so a paper claim needs n=3 both-thread
characterization, but the direction is clear: DO NOT inject vision into the ambiguity marginal
covariance. Reinforces pivot to VIO-consistency weighting of raw GNSS (a different mechanism).
Runs: results/research/ablation_vision_t1/{va_on,va_off}/deep/.
