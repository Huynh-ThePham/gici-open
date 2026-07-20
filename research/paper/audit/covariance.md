# Covariance-extraction path audit (fast Q_aa vs ceres reference)

Scope: `src/estimate/graph.cpp` (`getMarginalAmbiguityCovariance`, `getLocalCrossInformation`,
`computeCovariance` + `extract_covariance`), `include/gici/estimate/error_interface.h`, and the
research diff (`git diff HEAD`). Cross-checked `include/gici/imu/imu_error.h` +
`src/imu/imu_error.cpp` for the suppression hook, and `gnss_parameter_blocks.h` for ambiguity
block dimensions.

Headline claim under test: fast `Q_aa` matches the exact ceres covariance to ~1e-3 in ~16x less
time. Verdict summary: **no CONFIRMED bug**. One **SUSPECTED** claim-scoping issue (the suppressed
IMU linearization redefines "exact ceres" reference); everything else **REFUTED**.

---

## 1. IMU relinearization suppression changes the reference covariance itself — SUSPECTED (low severity, claim-scoping)

- file:line: `src/imu/imu_error.cpp:598-605` (`if (redo_ && !suppress_relinearization_)`);
  invoked from `graph.cpp:1391-1399` (RelinGuard around `ceres::Covariance::Compute`) and
  `graph.cpp:496-500` (fast path).
- Defect: with `suppress_relinearization_ = true`, `ImuError::EvaluateWithMinimalJacobians` skips
  `redoPreintegration()` and returns the *first-order bias-corrected* Jacobian/residual about the
  last linearization point instead of the freshly re-preintegrated one. The RelinGuard applies this
  to the ceres reference computation as well. So the benchmark's "exact ceres covariance"
  (`computeCovariance`) is evaluated at the suppressed (first-order) IMU linearization, NOT what a
  vanilla `ceres::Covariance::Compute` (no suppression) would produce when any IMU factor's bias
  drift has latched `redo_ = true`.
- Concrete trigger: any AR epoch where an in-window IMU factor has accumulated bias drift past the
  redo threshold `Delta_b.head<3>().norm() * delta_t > 0.0001` (line 601 sets `redo_` even under
  suppression). At such an epoch, vanilla-ceres reference != suppressed-ceres reference by the
  higher-order preintegration terms.
- Refutation attempt: does this break the paper's fast≈ceres number? No — BOTH the fast path and the
  benchmark ceres path go through the identical suppressed evaluation, so they agree with each other
  (the reported `rel_err` is between two internally-consistent quantities). The mechanism is what
  MAKES them consistent, not what breaks them. Survives only as a *definition* issue: the object the
  paper calls the "exact Ceres covariance" is the suppressed-linearization covariance. A reviewer
  re-running stock `ceres::Covariance` without the hook, on a drifted graph, would see a small extra
  discrepancy relative to the fast Q_aa. Typically tiny (first-order correction is accurate for
  sub-threshold drift), so severity is low, but it is real and touches the headline claim's wording.
- Minimal suggested fix (do NOT apply): in the paper, state that both estimators are compared at the
  graph's current (post-optimize) linearization with IMU preintegration held fixed — i.e. define the
  reference as the covariance at the optimizer's own linearization point — rather than claiming
  equality to an unqualified "exact Ceres covariance". No code change needed; the code is
  self-consistent by construction.

## 2. Index/ordering mismatch between fast Q_aa and ceres block layout — REFUTED

- file:line: `graph.cpp:355-362` / `394-395` (fast, A-partition offset = caller index `k`) vs
  `graph.cpp:1309-1359` (ceres, `parameter_block_starts`/`extract_covariance`); consumer
  `rtk_imu_camera_rrr_va_estimator.cpp:62-70,104-110`.
- Defect hypothesis: rows/cols of fast and ceres could be permuted, inflating/masking `rel_err`.
- Refutation: both consume the same `parameter_block_ids` vector, built once from
  `curAmbiguityState().ids` in a single ordered loop. Fast path assigns ambiguity `k` to
  `Slot{A, k, 1}`, so `H_aa`/`Q_aa` row `k` = ambiguity `k`. Ceres path builds `parameters[i]` in the
  same order and writes `output.block(start_i,start_j,...)` at cumulative offsets. Ambiguity blocks
  are `CommonParameterBlock<1,...>` (ambient=minimal=1, confirmed in `gnss_parameter_blocks.h:21`),
  so `parameter_size == num_amb`, `start_i == i`, and both matrices are `num_amb x num_amb` in
  identical order. `rel_err` compares aligned entries. Refuted.

## 3. Information vs covariance confusion (returning S instead of S^{-1}, etc.) — REFUTED

- file:line: `graph.cpp:627-643` (fast) and `graph.cpp:1345-1357` (ceres).
- Defect hypothesis: fast path returns an information matrix while ceres returns covariance.
- Refutation: fast path builds `S` = reduced *information* (Schur complement of the info matrix,
  line 629 `S = H_aa - T`), then explicitly inverts via eigen-reciprocal
  (`Q_aa = V * (1/λ) * V^T`, lines 641-642) → covariance. Ceres `GetCovarianceBlock` returns
  covariance directly. Both are covariances. Trace comparison in the estimator (`tr_fast`,
  `tr_ceres`) would diverge by orders of magnitude if one were an information matrix; the reported
  agreement confirms both are covariances. Refuted.

## 4. Wrong sign/scale in the Schur complement or equilibration — REFUTED

- file:line: `graph.cpp:598-629`.
- Defect hypothesis: Jacobi equilibration `D^{-1} H_nn D^{-1}` might not cancel in the Schur term,
  leaving a scale error; or `H_an H_nn^{-1} H_na` sign wrong.
- Refutation: `rhs_s = D^{-1} H_an^T`, `Hs = D^{-1} H_nn D^{-1}`, `Y = Hs^{-1} rhs_s = D H_nn^{-1} H_an^T`,
  `T = rhs_s^T Y = H_an D^{-1} D H_nn^{-1} H_an^T = H_an H_nn^{-1} H_an^T` — the `D` factors cancel
  exactly as the comment claims. Sign is `S = H_aa - T` (correct Schur subtraction). Off-diagonal
  `H_an` is assembled once from the (A,N) ordering (line 421-422) and reused transposed as `H_na`;
  the dropped (N,A) branch (line 432) is intentional and correct because only `H_an` is needed.
  Refuted.

## 5. Off-by-one / wrong block extraction in landmark Schur strips — REFUTED

- file:line: `graph.cpp:397-441` (strip assembly) and `graph.cpp:552-576` (stage-1 elimination).
- Defect hypothesis: landmark coupling strips (`dim_n x 3`) mis-sized or scattered to wrong N offset.
- Refutation: `strip(off, dim)` allocates `dim x 3` keyed by the N-block offset; scatter routes
  `(N,L)` C = `Jn^T Jl` (`dim_n x 3`) into `lms[l].strip(n_off, dim_n)` (line 430). Stage 1 forms
  `Bi * pinv(Hll) * Bj^T` (`dim_i x dim_j`) and subtracts from `H_nn.block(off_i,off_j,...)`
  (line 572-573) — dimensions and offsets are consistent. `(L,L)` requires equal offsets else
  `structure_ok=false` (line 435). `(A,L)`/`(L,A)` set `structure_ok=false` (lines 424,438) and the
  routine aborts via `partition_violated` (line 536) rather than mis-assembling. Rank truncation uses
  eps-relative tol on `H_ll` eigenvalues (Moore-Penrose drop of null modes), which is the exact
  marginalization of an unobservable landmark direction. Refuted.

## 6. Marginalization prior handled inconsistently between fast and ceres — REFUTED

- file:line: `graph.cpp:324-338` (pass 0), `451-470` (prior scatter) vs ceres operating on `problem_`.
- Defect hypothesis: fast path might drop or double-count the prior relative to ceres.
- Refutation: fast path reads prior info once (`marginalizationInformation`) and scatters
  `prior_Lambda` blocks directly, with a layout guard (`prior_layout_mismatch`, line 458) and
  multiple-prior guard (line 331). Prior-touched blocks are excluded from landmark classification
  (`prior_id_set`, line 373) so they are eliminated densely in N — consistent with how ceres treats
  them. Ceres `computeCovariance` runs on `problem_`, which contains the same
  `kMarginalizationError` residual. Both include the prior exactly once. (Note the *ablation-only*
  `getLocalCrossInformation` deliberately excludes the prior, line 212, but that path is not the
  default VA path and is not the one benchmarked against ceres.) Refuted.

## 7. Stale factorization reuse across epochs — REFUTED

- file:line: `graph.cpp:1380` (`ceres::Covariance covariance_handle(options)` local per call);
  fast path assembles H from scratch each call.
- Refutation: the ceres `Covariance` object and the LDLT factorization (`graph.cpp:607`) are both
  constructed fresh inside each `computeCovariance` / `getMarginalAmbiguityCovariance` invocation.
  No cached factorization persists across AR epochs. Refuted.

## 8. PSD / conditioning handling could fabricate or falsely validate — REFUTED

- file:line: fast gates `graph.cpp:635-644`; ceres path `rtk_imu_camera_rrr_va_estimator.cpp:104-128`.
- Defect hypothesis: a rank-deficient graph could make fast and ceres silently disagree while
  reporting a small `rel_err`.
- Refutation: fast path applies a strict PD gate `minCoeff <= max(eps*n*maxev, arith_err)` →
  `final_below_arith_err` abstain (returns false); the estimator then does not compare (guarded by
  `ok && ceres_ok`, line 104) and falls back to shadow covariance (line 171). When fast succeeds the
  reduced information was PD, so ceres SPARSE_QR should also succeed on the same well-posed system.
  If ceres needs the DENSE_SVD/Moore-Penrose fallback (gauge/rank deficiency, line 1413-1420) the
  fast path would already have abstained. No path reports a small `rel_err` while comparing
  mismatched quantities. Refuted.

## 9. Loss-function (robustifier) correction inconsistent with ceres — REFUTED

- file:line: `graph.cpp:502-519` (Triggs/BANS correction in fast path).
- Defect hypothesis: fast path applies robust-loss reweighting that ceres::Covariance does not (or
  vice versa), biasing the comparison.
- Refutation: the fast path replicates ceres' `corrector.cc` Triggs correction
  (`sqrt(rho1)`, `alpha_sq_norm`) exactly, matching the robustified Hessian ceres::Covariance builds
  from the same loss. Both include the loss. Any mismatch would show as a systematic `rel_err`
  offset; the empirical ~1e-3 agreement is consistent with matched corrections. Refuted.

## 10. Thread-safety of the mutable suppress flag / non-RAII restore leak — REFUTED

- file:line: `imu_error.h:333` (`mutable bool suppress_relinearization_`); set/restore at
  `graph.cpp:260-265`, `496-500`, `1391-1399`.
- Defect hypothesis: (a) concurrent covariance queries race on the flag; (b) an early return/throw
  between set(true) and set(false) latches suppression true, corrupting the next optimize().
- Refutation: (a) the fast and ceres computations run sequentially within
  `estimateVisionAidedAmbiguityCovariance` (t0..t1 then t2..t3); no concurrent covariance queries.
  ceres::Covariance may internally multithread residual evaluation, but the flag is set before
  `Compute` and read-only during it (the whole point is to prevent the in-place mutation that would
  otherwise be the race). (b) In both fast paths the set(false) at lines 265/500 immediately follows
  the single `Evaluate` with no return/throw in between (the loss correction and scatter run after
  restore). `computeCovariance` uses an RAII `RelinGuard` (lines 1391-1399), exception-safe on every
  exit. No leak. Refuted.
  - Minor robustness nit (not a bug): the fast-path per-residual set/restore is manual rather than
    RAII; harmless today because nothing between the calls can early-return, but an RAII guard would
    be more future-proof.

---

Bottom line: the covariance-extraction paths are internally consistent and the fast/ceres
comparison is apples-to-apples (same ordering, both covariances, both including prior+loss, both at
the suppressed IMU linearization). The only item worth acting on is #1 — a wording/scoping
clarification of what "exact Ceres covariance" means, not a code defect.
