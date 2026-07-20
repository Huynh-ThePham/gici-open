# Marginalization / error-computation subsystem — static audit

Scope: `src/estimate/estimator_base.cpp` (marginalization ordering guard),
`include/gici/estimate/error_interface.h`, `src/estimate/graph.cpp`
(marginalization paths), `src/estimate/marginalization_error.cpp` +
`include/gici/estimate/marginalization_error.h`. Read fully. Driving estimator
path (`src/fusion/rtk_imu_camera_rrr_estimator.cpp`) inspected read-only only to
construct/refute triggers; it is out of the reportable scope.

Method: every candidate below was pushed through an explicit refutation attempt
(name a concrete epoch/state configuration, then check whether another code path
prevents it). Default verdict is REFUTED unless a surviving concrete trigger
exists.

---

## Recently-fixed ordering guard (estimator_base.cpp:104) — VERIFIED CORRECT (residual concern noted)

file: `src/estimate/estimator_base.cpp:104-106`

The guard was changed from `parameter_blocks_to_be_marginalized.size() > 0` to
`marginalization_error_->parameterBlocks() > 0` before gating
`updateErrorComputation()`. This is not re-reported as a new bug; verification of
the fix follows.

Correctness of the fix:
- Both mutators of the prior set the invalidation flag:
  `addResidualBlock` (marginalization_error.cpp:151) and `marginalizeOut`
  (marginalization_error.cpp:676) both set `error_computation_valid_ = false`.
- `applyMarginalization` can add residuals to the prior (loop at
  estimator_base.cpp:77-82, and the `add*MarginBlocks*` calls in the estimator)
  on epochs where **zero** parameter blocks are pushed into
  `marginalization_parameter_ids_`. `marginalizeOut` then early-returns at
  marginalization_error.cpp:556 (`parameter_block_ids.size()==0 -> return
  false`) WITHOUT further touching the flag. Under the OLD guard, a prior whose
  `H_`/`b0_` had just been mutated by `addResidualBlock` was re-added to the
  graph with a stale `J_`/`e0_`, and the next `Evaluate` hit the fatal
  `CHECK(error_computation_valid_)` at marginalization_error.cpp:1043. The
  concrete trigger (harsh urban, cycle-slip/landmark-loss epoch that appends IMU
  or GNSS residuals but marginalizes no state) is real and matches the
  Harsh-only intermittence recorded.
- The new guard keys on `parameterBlocks()` (=
  `base_t::parameter_block_sizes().size()`), which is non-zero whenever any block
  has been added, so it fires on exactly those "residuals appended, nothing
  marginalized" epochs the old guard skipped.
- Behaviour-preserving for previously-working epochs: `updateErrorComputation()`
  self-guards (`if (error_computation_valid_) return;` at
  marginalization_error.cpp:900), so calling it when already valid is a genuine
  no-op. It only recomputes `S_/S_pinv_/J_/e0_` from the existing `H_/b0_`; it
  changes no estimator state, weight, or linearization point.
- The empty-prior case is handled downstream: if `parameterBlocks()==0` the guard
  skips recompute, and estimator_base.cpp:109 (`num_residuals()==0`) then resets
  the prior, so no stale prior is ever re-added.

Verdict: the fix is correct and addresses the stated defect. See the SUSPECTED
item below for the one residual edge it does not fully cover.

---

## Empty-H_ eigen-solve when all prior blocks are fixed — SUSPECTED (very low likelihood, benign under NDEBUG)

file: `src/estimate/marginalization_error.cpp:916-920`; guard at
`src/estimate/estimator_base.cpp:104`

Defect: the guard uses `parameterBlocks() > 0`, but `parameterBlocks()` counts
parameter-block-info entries **including fixed blocks whose `minimal_dimension`
is 0** (fixed blocks are still appended to `parameter_block_infos_` and to
`base_t::parameter_block_sizes()` at marginalization_error.cpp:229-272,
independent of the `additionalSize>0` test). Those blocks contribute 0 columns to
`H_`. So it is representable to have `parameterBlocks() > 0` while `H_.cols()==0`.
In that state the guard calls `updateErrorComputation()`, which builds a
`SelfAdjointEigenSolver` on a 0x0 matrix and calls
`saes.eigenvalues().array().maxCoeff()` on an empty array
(marginalization_error.cpp:920).

Trigger scenario: a prior all of whose connected parameter blocks are fixed at
the same time, with no non-fixed block ever having contributed to `H_`.

Refutation attempt (why it mostly survives but is downgraded): to reach
`H_.cols()==0` with `parameterBlocks()>0`, the prior must have accumulated only
fixed (zero-minimal-dim) blocks since construction — `addResidualBlock` with an
all-fixed block leaves `additionalSize==0` and does not grow `H_`. In the real
pipeline the first marginalization always marginalizes genuine non-fixed states
(IMU/pose/ambiguity), so `H_.cols()>0` holds whenever `parameterBlocks()>0`. I
could not construct a run-realistic epoch that fixes every previously-non-fixed
prior block simultaneously. Additionally, under `-DNDEBUG` Eigen's `eigen_assert`
is compiled out, so `maxCoeff()` on the empty array yields `-inf`, `tolerance`
becomes `nan`, the `.select(...)` produces empty `S_`/`J_`/`e0_`,
`num_residuals()` becomes 0, and estimator_base.cpp:109 then resets the prior —
i.e. no crash in release, only a debug-build assertion. Hence SUSPECTED, not
CONFIRMED.

Suggested minimal fix (do not apply): make the guard defensive against the
zero-dimension case by keying on the linear-system size instead of the block
count, e.g. `marginalization_error_->residualDim() > 0` (==
`base_t::num_residuals()`, which equals `H_.cols()` at that point since
`set_num_residuals(H_.cols())` is kept in sync by both `addResidualBlock` and
`marginalizeOut`). This is strictly stronger than the current guard and still
fires on every trigger the current fix targets.

---

## Stale J_/e0_ reused while error_computation_valid_ is true but H_ changed — REFUTED

file: `src/estimate/marginalization_error.cpp:900-903`

Candidate: `EvaluateWithMinimalJacobians` trusts `error_computation_valid_`; if
`H_`/`b0_` were mutated without clearing it, a stale decomposition would be used.

Refutation: every mutation of `H_`/`b0_` sets `error_computation_valid_ = false`
in the same function — `addResidualBlock` (line 151, before any resize/insert)
and `marginalizeOut` (line 676, before the Schur ops). There is no other writer
of `H_`/`b0_`. `updateErrorComputation()` is the only place the flag is set true,
and it does so only after rebuilding `J_`/`e0_`. No path mutates the system and
leaves the flag true. Refuted.

---

## Prior routed through a dynamically-sized generic evaluation buffer — REFUTED

file: `src/estimate/graph.cpp:307-534` (`getMarginalAmbiguityCovariance`),
`:176-286` (`getLocalCrossInformation`)

Candidate: the historically fragile heap overflow was assembling the
marginalization prior through the per-residual raw-buffer
`EvaluateWithMinimalJacobians` path (its residual dim = `H_.cols()`, dynamic).

Refutation: both research routines deliberately avoid that path.
`getLocalCrossInformation` skips `kMarginalizationError` entirely
(graph.cpp:212). `getMarginalAmbiguityCovariance` reads the prior once via
`MarginalizationError::marginalizationInformation()` (graph.cpp:334), which
returns `Lambda = J_^T J_` plus the non-fixed block ordering, and scatters those
sub-blocks by id (graph.cpp:451-471) — it never calls `Evaluate` on the prior.
Layout is bounds-checked: `marginalizationInformation` rejects out-of-range
ordering (marginalization_error.cpp:979) and skips `minimal_dimension==0` blocks
(line 977); the scatter loop rejects any per-block dim mismatch
(`prior_layout_mismatch`, graph.cpp:458). Ordinary residuals are sized per
`residualDim()`/`dimension()`/`minimalDimension()` with matching Eigen buffers
(graph.cpp:486-491). Refuted.

---

## Thread-safety of error_computation_valid_ / J_ under num_threads>1 — REFUTED

file: `include/gici/estimate/marginalization_error.h:364`; `graph.cpp` covariance
/ information routines; `optimize()` in estimator_base.cpp:30-55

Candidate: with `graph_->options.num_threads > 1`, Ceres evaluates residual
blocks concurrently; `error_computation_valid_` (volatile bool) and `J_`/`e0_`
could be read/written from multiple threads.

Refutation: during a solve, `MarginalizationError::EvaluateWithMinimalJacobians`
only READS `error_computation_valid_`, `J_`, and `e0_`; it writes nothing on the
object (outputs go to caller-owned ceres buffers). All writers
(`updateErrorComputation`, `addResidualBlock`, `marginalizeOut`) run
single-threaded in `applyMarginalization`, strictly before `optimize()`/`solve()`
is entered. Ceres partitions distinct residual blocks across threads, so no two
threads evaluate the same factor object concurrently. `setSuppressRelinearization`
toggles a mutable flag but is used only inside the single-threaded, post-solve
covariance/information assembly (graph.cpp:260-265, 496-500, 1391-1399), never
during the multithreaded solve. No data race with a concrete trigger. Refuted.

---

## num_residuals vs e0_ dimension mismatch handed to Ceres — REFUTED

file: `src/estimate/marginalization_error.cpp:906, 1093-1094`;
`estimator_base.cpp:117`

Candidate: Ceres reads `num_residuals()` when the prior is added
(estimator_base.cpp:117) and allocates residual buffers of that size; if `e0_`
had a different length, `EvaluateWithMinimalJacobians` (writing `e0_.rows()`)
would overflow.

Refutation: `updateErrorComputation` sets `num_residuals := H_.cols()`
(line 906) and computes `e0_ = -J_pinv_T * b0_`, whose length equals `H_.cols()`
(the eigen-decomposition is of the `H_.cols()`-square `H_`). Both `addResidualBlock`
(line 301) and `marginalizeOut` (line 822) keep `num_residuals` consistent with
`H_.cols()` at their own exit, and the prior is (re-)added to the graph only after
`updateErrorComputation` has run for the current shape. `residualDim()` returns
`num_residuals()`, so Ceres' allocation and the `Eigen::Map<VectorXd> e(residuals,
e0_.rows())` write agree. Refuted.

---

## Summary

No new CONFIRMED bug found in the in-scope marginalization / error-computation
code. The already-applied ordering-guard fix at estimator_base.cpp:104 is correct
and does resolve the stated intermittent fatal CHECK. One SUSPECTED residual edge
remains (all-prior-blocks-fixed => empty `H_` eigen-solve): practically
unreachable in the pipeline and benign under NDEBUG, but a strictly-stronger guard
(`residualDim() > 0`) would close it. Four further candidates (stale-decomposition
reuse, dynamic-buffer prior overflow, num_threads race, e0_/num_residuals
mismatch) were each refuted with a concrete code path.
