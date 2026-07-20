# Ambiguity-resolution static audit (VA-AR path)

Scope: `src/gnss/ambiguity_resolution.cpp`, `src/gnss/ambiguity_resolution_differential.cpp`
(the RTK path our experiments use), `src/gnss/ambiguity_common.cpp` (+ headers), the vendored
LAMBDA (`third_party/rtklib/src/lambda.c`), the covariance that feeds AR
(`RtkImuCameraRrrVaEstimator::estimateVisionAidedAmbiguityCovariance`,
`Graph::getMarginalAmbiguityCovariance`), and the RTK ambiguity/cycle-slip bookkeeping in
`gnss_estimator_base*.cpp`. READ-ONLY; nothing was edited.

Verdict summary: 1 CONFIRMED logic defect (benign to the paper claim, upstream/pre-existing),
1 SUSPECTED low-severity robustness gap, everything else REFUTED. **No bug found that
fakes or hides the vision-aiding effect.** The core mechanism (vision/IMU tightening the
ambiguity marginal covariance) is implemented in the correct direction and is backstopped by
a post-fix range-cost check.

---

## Partial-AR "Variance" subset selection sorts the wrong way — CONFIRMED (benign, upstream)
- File: `src/gnss/ambiguity_resolution.cpp:530-533` (sort) with `:621-622` (reduction) and
  `:598-623` (LAMBDA partial loop); guard at `:612`.
- Defect: partial AR keeps `float_ambiguities.topRows(num_active)` and drops the tail as
  `num_active` shrinks. So the *tail* must hold the least-reliable ambiguities. The three
  `DelMethod` sorts are inconsistent:
  - `Elevation` sorts **descending** by elevation → worst (low-elevation) at tail → dropped.
    Correct.
  - `Fractional` sorts **ascending** by |cycle-round(cycle)| → worst (largest fractional) at
    tail → dropped. Correct.
  - `Variance` sorts **descending** by `std` (`lhs.std > rhs.std`) → largest-variance
    (worst) at the *head*, smallest-variance (best) at the *tail* → the reduction drops the
    **best** ambiguities and keeps the **worst**. Backwards; should be ascending.
  - Side effect: the shrink guard `sqrt(active_float_covariance(num_active-1,num_active-1))
    < 0.25` then inspects the *smallest*-variance kept element instead of the worst, so it is
    effectively a no-op for this method.
- Concrete trigger: `trySolveLanes` runs {Elevation, Variance, Fractional}; when Elevation
  (full + all partial subsets) fails, the Variance stage runs with `skip_full=true` and
  immediately reduces. It retains the highest-variance subset.
- Refutation attempt: could this fabricate the VA fix-rate? A higher-variance subset has a
  *lower* LAMBDA ratio, so it is *harder* to pass the ratio test — the Variance stage
  therefore produces essentially no fixes (rather than false ones), and any freak fix is
  caught by the post-fix range-cost check (`_differential.cpp:195-204`). It reduces AR
  efficiency slightly and does so identically for baseline and VA runs, so it cannot inflate
  or hide the vision effect. Also confirmed upstream: `git log -S` attributes the line to the
  initial import commit `c785e27 "Upload."`, and the file is unmodified in the working tree.
- Survives as a genuine logic bug, but impact-neutral to the paper's claim.
- Minimal suggested fix (do NOT apply): change the `Variance` comparator to ascending
  (`return lhs.std < rhs.std;`) so the worst-variance lanes fall to the tail and are dropped
  first, matching the other two methods and partial-AR theory. Do not apply without a
  before/after fix-rate regression on all datasets.

## `lambda()` return status ignored → possible garbage ratio on LD failure — SUSPECTED (low)
- File: `src/gnss/ambiguity_common.cpp:356-382` (`solveAmbiguityLambda`), specifically the
  discarded return of `lambda(...)` at `:372-373` and `Eigen::Vector2d residuals;` at `:370`.
- Defect: `residuals` (the `s` output of `lambda`) is left uninitialized and `lambda`'s
  integer status is not checked. If LAMBDA's LD factorization fails (non-PD `Q`, e.g. a
  numerically singular `active_float_covariance`), `lambda` returns non-zero and never writes
  `s`, so `ratio = residuals[1]/residuals[0]` is computed from stack garbage and could exceed
  the ratio threshold, returning `true` with `fixed_ambiguities` = the zero-initialized
  `fixed_template.leftCols(1)`.
- Concrete trigger: a rank-deficient float covariance reaching LAMBDA. The only pre-checks in
  `solveLanes` are `sqrt(diag) < 0.25` (`:612`) and the success-rate gate (opt-in, default
  off) — neither guarantees positive-definiteness.
- Refutation attempt: (a) `float_covariance = J * ambiguity_covariance_ * Jᵀ` is PSD by
  construction and, for the VA path, `ambiguity_covariance_` is explicitly PD-gated
  (eigenvalue floors in `estimateVisionAidedAmbiguityCovariance` and
  `getMarginalAmbiguityCovariance`), so a true LD failure is rare; (b) even on a spurious
  "fix" the integers are all zero and the constrained re-solve's range-cost check
  (`_differential.cpp:195-204`, tol 0.01) rejects it. So it cannot silently corrupt results
  and is not a plausible source of the observed fix-rate. It survives only as latent
  fragility (uninitialized read), not a result-affecting bug; upstream and symmetric across
  variants.
- Minimal suggested fix (do NOT apply): capture the `lambda` return; on non-zero, set
  `ratio = 0` and return `false`. Zero-initialize `residuals`.

---

## REFUTED candidates (checked and dismissed)

- **Ratio-test threshold / inequality direction** — `ambiguity_common.cpp:374-381`:
  `ratio = s[1]/s[0]` (second-best over best squared residual), accept iff
  `ratio > ratio_threshold` (`options_.ratio`, default 3.0). Standard and correct direction.
  `s[0]==0` maps to `ratio=0` → reject (a degenerate perfect fit is conservatively dropped).
  Not a bug.

- **Cycle-slip flag not resetting ambiguity state** — `gnss_estimator_base_differential.cpp:
  87-193` (`addSdAmbiguityParameterBlocks`) and `ambiguity_common.cpp:44-142`
  (`cycleSlipDetectionSD` sets `slip` on rover+ref via LLI/GF/time-gap). Ambiguity parameter
  blocks are created fresh every epoch (`createGnssAmbiguityId(prn,phase,id)` with per-epoch
  bundle `id`). On slip the between-epoch time constraint is skipped (`:167-192`, `has_last`
  stays false → fresh initial prior added), so a slipped satellite is correctly reinitialized
  rather than carried forward. No stale/frozen ambiguity survives a slip. Not a bug.

- **Ambiguity state not marginalized/reinitialized on slip** — same mechanism as above; per-
  epoch blocks mean there is no persistent ambiguity value to marginalize incorrectly. The
  outage guards at `rtk_imu_camera_rrr_estimator.cpp:153` and `:211-218` (require
  `gnss_measurement_pairs_.size() > 1`) correctly skip cross-epoch constraints when history
  is absent (documented crash fix). Not a bug.

- **Reference-satellite / double-difference bookkeeping** — `formSatellitePairRtk`
  (`ambiguity_resolution_differential.cpp:213-367`) selects the max-elevation satellite with
  maximal phase count per system as BSD reference (standard). The DD phaserange residuals are
  formed independently in `formPhaserangeDDPair`; AR's BSD is a consistent re-pairing.
  `findMatch` (the only reference-satellite-swap bookkeeping, `ambiguity_resolution.cpp:
  798-894`) is **dead code** (no caller; `grep` finds only its definition) and
  `addStableFixationToGraph` is declared but never defined or called — so no
  reference-swap bug is reachable. Not a bug.

- **Covariance/ordering mismatch between AR and its input covariance** —
  `estimateVisionAidedAmbiguityCovariance`, `estimateAmbiguityCovariance`, and
  `solveRtk` all iterate `curAmbiguityState().ids` in the same order; `solveRtk`'s
  `ambiguity_index_map` re-maps filtered→raw indices consistently
  (`ambiguity_resolution_differential.cpp:40-113`). Row/col order is preserved. Not a bug.

- **Float→fixed constraint applied with wrong covariance** — `ambiguity_resolution.cpp:
  661-671, 713-721`. Hard constraint is `1e-6 cycle²` (std 0.001 cyc); the opt-in
  soft-weight path (`use_success_rate_fix_information`, default off) sets
  `variance = (1-Ps)*1 + Ps*1e-6`. Units are consistent (Jacobian coefficients `1/wavelength`
  map metre-valued ambiguity params to cycles; covariance in cycle²). The Kalman update of
  `ambiguity_covariance_` uses the matching `1e-6·I`. Not a bug; the soft-weight path can only
  *weaken* constraints (more conservative), never fabricate a tighter fix.

- **Integer-bootstrapping success-rate gate faking fixes** — `bootstrapSuccessRate`
  (`ambiguity_resolution.cpp:965-1035`) and its use at `:604-617`. Both entry points
  (`min_bootstrap_success_rate>0`, `use_success_rate_fix_information`) default off, and when
  on they only *reject* low-Ps subsets or *inflate* the fixed-constraint variance — strictly
  more conservative. Cannot inflate the fix rate. Not a bug.

- **VA marginal covariance overconfident → fakes the effect** —
  `Graph::getMarginalAmbiguityCovariance` (`graph.cpp:307-646`) assembles the full
  Gauss-Newton information over ambiguities(A)/landmarks(L)/nuisance(N) including the
  marginalization prior, eliminates L then N, and returns
  `Q_aa = (H_aa - H_an H_nn⁻¹ H_naᵀ)⁻¹`. Adding vision/IMU *increases* `H_nn` (pose better
  constrained) → smaller Schur subtraction → larger reduced information → **smaller** `Q_aa`.
  This is the correct, statistically-consistent direction, matches the ceres full-covariance
  reference (built-in `benchmark_joint_ambiguity_covariance` rel-err check), and every stage
  has a PD/roundoff/arith-error gate that returns false (falling back to the GNSS-only shadow
  covariance) rather than fabricating confidence. A wrong integer that slipped through would
  still have to pass the post-fix range-cost check. Not a bug.
  - Caveat (methodological, not a code defect): the *baseline* covariance comes from a
    separate coarse GNSS-only shadow `RtkEstimator`
    (`rtk_imu_camera_rrr_estimator.cpp:799-822`), while the VA covariance comes from the full
    joint graph. Part of any measured gain is "full-graph marginal vs coarse shadow", not
    purely "add vision". Worth an ablation (`ablate_reprojection_in_ar`) note in the paper;
    it is not an AR-path bug.

- **Thread-safety** — AR, both covariance paths, and the shadow estimator's `estimate()` all
  run synchronously inside the estimator's `estimate()`/`addMeasurement()` call chain; the
  only mutex (`imu_mutex_`) guards the IMU buffer, not ambiguity state. `getMarginalAmbiguity
  Covariance` toggles `setSuppressRelinearization` around evaluation to avoid perturbing the
  graph linearization, and restores it. No concurrent access to ambiguity/graph state in the
  AR path. Not a bug.
