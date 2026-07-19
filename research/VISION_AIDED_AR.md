# Vision-Aided Ambiguity Resolution — research branch

**Branch:** `research/vision-aided-ambiguity-resolution`
**Worktree:** `/home/theph/ws_ncs/gici_vision_aided_ar`
**Base:** `research/standard-env` (upstream-clean `f2b8579` + research wrappers only —
deliberately *not* branched from the ROS2 real-time work, since this is a file-mode
algorithmic study that must stay comparable to the paper's own evaluation methodology).
**Goal:** evaluate a statistically defensible way to use joint visual/IMU/GNSS
information in GICI-LIB's RTK ambiguity resolution (AR), without contaminating locked
baselines or reporting tuned/partial results as wins.

**Current verdict (2026-07-18):** the branch now has a covariance path that is BOTH
statistically consistent AND real-time: `Graph::getMarginalAmbiguityCovariance` (see the
2026-07-18 section below) computes the exact-in-window marginal ambiguity covariance —
marginalization prior included, all nuisance states marginalized, loss-corrected — at
mean 28 ms/epoch vs ceres' 812 ms, validated per-epoch against `ceres::Covariance` on the
full GICI-board 1.1 trajectory (90% of epochs within 1e-3; every disagreement in the
conservative-or-tied direction: 171/177 larger, 6 within 7.3e-4 trace at the 1e-3 boundary,
no material overconfidence). Frozen baselines preserved; VA on 1.1 shows
no regression. The Medium/Deep accuracy claim remains **PENDING** — UrbanNav data is not
present on this machine; do not cite urban accuracy numbers until it is rerun.

*(2026-07-17 verdict, superseded: the delta-information VA-AR covariance was not
accuracy-safe — on UrbanNav Deep it increased fixed-rate while degrading horizontal
accuracy. That overconfidence mechanism is exactly what the new path eliminates.)*

## Motivation (author-acknowledged gap)

Chi et al. 2023 (RAL), Section IV-E, describe AR as purely GNSS-domain: float ambiguities
and their covariance are extracted from the joint factor graph, a Between-Satellite-
Difference is applied, and MLAMBDA does the integer search — validated by a ratio test.
Nothing about visual/INS information enters the AR step itself, even though the *float*
estimate already comes from a joint GNSS+INS+camera optimization.

The paper's own discussion section states this directly:

> "the RRR estimator reaches the best performance in most of the trajectories due to
> the exhaustive utilization of multi-sensor measurements, but the improvement is not
> as much as one can expect relative to the TC estimator because of our greater
> tendency towards GNSS when we design the entire estimator. **One can explore the
> ability of visual estimation to further improve the performance of the
> visual-relevant estimators.**"
> — Chi et al. 2023, Section V-B

This is an author-acknowledged open problem, not a speculative gap.

## Empirical motivation from this repo

- UrbanNav Medium (this repo's own baseline work, 2026-07-16/17): RTK never reaches
  "fixed" ambiguity status — 100% float epochs (`fixed_rate: 0.0`) — even after fixing
  three real UB bugs and a base-RINEX sampling-rate issue that closed the accuracy gap
  to the paper. Float-only operation is exactly the regime where better AR matters most.
- Deep achieves "fixed" only ~1% of epochs (`fixed_rate: 0.009`) — also effectively an
  always-float regime in practice, despite very different vertical accuracy (1.9m vs
  Medium's — see `research/BASELINE_LOCK.md`), showing float/fixed rate alone is a
  crude proxy for the underlying uncertainty AR should be exploiting.

## Related work (checked 2026-07-17 — this is an active, not empty, research area)

| Paper | Mechanism | How this differs |
|---|---|---|
| Ran et al., "A VIO-aided partial ambiguity resolution for RTK positioning in complex urban environments," *GPS Solutions* 30, 34 (2026) | VIO-**predicted position** used as an external side-channel: ranks/selects satellites for partial AR via DD pseudorange/phase residuals computed against the VIO prediction; smooths pseudorange; flags false-fix in the position domain post-hoc. | VIO is a separate, loosely-coupled auxiliary estimate feeding satellite *selection* and post-hoc validation — not the joint optimization's own statistical structure. |
| (2024) "A vision-aided RTK ambiguity resolution method by map lane matching for intelligent vehicle in urban environment," *J. Spatial Sci.* | Uses an HD map of lane-line geometry + vision-based relative positioning to constrain position/yaw before single-epoch AR. Requires a pre-built lane map. | Map-dependent; not applicable without HD maps; different information source entirely (map priors, not VIO uncertainty). |
| "3D Vision Aided GNSS RTK Positioning for Autonomous Systems in Urban Canyons," *NAVIGATION* 70(3) (2023) | 3D building/semantic vision aids NLOS/multipath satellite rejection before RTK, not AR search/validation directly. | Different problem (signal quality gating, not the integer search itself). |
| GS-GVINS, PO-GVINS, and other 2024-2025 GNSS-visual-inertial factor-graph systems | General tightly-coupled GVINS with AR "inside" the graph (ambiguities as state nodes), but AR itself (decorrelation + search + validation) still operates on the GNSS-only marginal, as far as public descriptions show. | Same gap as GICI-LIB itself — none reported exploiting the joint Hessian's ambiguity/pose cross-covariance specifically. |

**Novelty claim:** GICI-LIB already estimates GNSS ambiguities and INS/camera states in
*one* joint FGO problem (Eq. 1 of the paper) — meaning a statistically rigorous
cross-covariance between float ambiguity states and pose/velocity states is *latent*
in the same Hessian already being formed. Exploiting it (rather than an external VIO
side-channel, as in Ran et al. above) is a mechanism specific to genuinely
tightly-coupled FGO architectures like GICI-LIB's — not reported in the papers found
in the literature pass above.

### Confirmed against the actual code (2026-07-17 deep-dive) — the gap is real, and why it exists

`RtkImuCameraRrrEstimator` (the vision-tightly-coupled estimator — `src/fusion/
rtk_imu_camera_rrr_estimator.cpp`) does **not** use the joint graph's own covariance
for AR at all. It maintains a second, fully separate, GNSS-only shadow estimator,
`ambiguity_covariance_estimator_` (`include/gici/fusion/rtk_imu_camera_rrr_estimator.h:112`,
constructed with AR disabled and a 2-epoch window, `rtk_imu_camera_rrr_estimator.cpp:50-59`).
Every GNSS measurement is fed to *both* the real joint graph and this shadow problem in
parallel (lines 90-99); `estimateAmbiguityCovariance()` (lines 766-789) pulls the
ambiguity covariance from the **shadow graph**, never the joint one. The code's own
comment states the reason directly:

> "Computing the ambiguity covariance directly is very time-consuming, so we run a
> parallel optimizer and forcely set some empirical constraints to estimate a coarse
> covariance." — `rtk_imu_camera_rrr_estimator.cpp:769-770`

More generally, `Graph::computeCovariance()` (`src/estimate/graph.cpp:822-884`) only
ever forms covariance blocks for whatever parameter-block IDs the caller supplies —
every AR call site (PPP/RTK/TC/RRR estimators) supplies *only* the current ambiguity
IDs (e.g. `curAmbiguityState().ids`), never pose/velocity/attitude IDs. So the gap is
confirmed exactly as hypothesized, and the original authors' own code comment confirms
*why*: they judged the full joint covariance (spanning the whole sliding window —
poses, velocities, biases, visual landmarks, ambiguities) too expensive for real-time
use, and chose a decoupled workaround instead of a partial one.

**This reframes the contribution precisely:** not "add cross-covariance nobody
thought of," but "find a computationally tractable middle ground the original authors
didn't take."

### Cost measurement result (2026-07-17) — the naive approach is confirmed infeasible

Instrumented `rtk_imu_camera_rrr_estimator.cpp` (option `benchmark_joint_ambiguity_covariance`,
default off) to time `graph_->computeCovariance()` for the ambiguity IDs *plus only the
current epoch's pose/speed-and-bias block* (not the full window) against the existing
shadow-estimator path, on UrbanNav Medium (100% float — 641 logged epochs, full
trajectory):

| Graph size (parameter blocks) | Joint-graph query (mean / max) | Shadow estimator (mean) |
|---|---|---|
| < 100 (early, window still filling) | 1.0 ms / 1.6 ms | 0.2 ms |
| 200-400 (steady state — most of the run) | 500-2000+ ms (max 2.28 **s**) | 0.2-0.7 ms |
| **Overall** | mean 687 ms, median 653 ms, **86.4% of epochs > 50 ms** | mean 0.39 ms |

The cost does not scale with the *number of blocks requested* (always ambiguities +
2 blocks) — it scales with the graph's *total* size, confirming this is dominated by
Ceres's covariance algorithm needing to factor structure tied to the whole problem,
not by how many blocks are ultimately extracted. **The naive "just query the joint
graph for a couple more blocks" approach is confirmed infeasible for real-time (or
even reasonably-paced batch) use** — this quantitatively confirms and explains the
original authors' shadow-estimator workaround, rather than revealing an oversight.

### Revised technical direction

Don't call `ceres::Covariance` (a whole-problem algorithm) at all. Instead, use
`ceres::Problem::Evaluate()` restricted to only the *current epoch's newly-added*
residual blocks (pseudorange/carrier-phase/Doppler, the IMU pre-integration factor to
the previous pose, and visual reprojection factors for the current keyframe) to get
their Jacobians, form a small local information matrix (`J^T J`, on the order of a few
tens of dimensions — ambiguities + pose/velocity/bias) directly with Eigen, and invert
that small block analytically. Cost should depend only on the number of *new* residuals
per epoch (bounded, small), not on accumulated graph/window size — matching the
shadow-estimator's own bounded-cost design principle, but using real joint information
instead of a decoupled GNSS-only shadow problem. This needs its own cost measurement
before trusting it, same as the first approach.

Natural entry point: still `rtk_imu_camera_rrr_estimator.cpp:387`-ish, replacing the
call into `ambiguity_covariance_estimator_`/`estimateAmbiguityCovariance()` — but the
new implementation queries `graph_`'s current-epoch residual blocks directly rather
than either the shadow graph or a naive full-block `computeCovariance()` call.

### Local Jacobian approach — cost measurement result (2026-07-17) — confirmed tractable

Implemented `Graph::getLocalCrossInformation()` (`include/gici/estimate/graph.h`,
`src/estimate/graph.cpp`): builds on the existing (previously unused/dead)
`Graph::getLhs()` pattern — evaluates only the residual blocks touching the requested
parameter blocks directly via `ErrorInterface::EvaluateWithMinimalJacobians`, no
`ceres::Covariance`/`Problem::Evaluate` involvement — and generalizes it to accumulate
*cross* terms between multiple requested blocks (ambiguities + current pose +
speed-and-bias), not just one block's diagonal.

Measured the same way as the first (infeasible) approach, on the same UrbanNav Medium
run (640 epochs):

| Graph size (parameter blocks) | Local-information query (mean / max) |
|---|---|
| < 100 | 0.15 ms / 0.44 ms |
| 200-300 (steady state) | 0.19 ms / 0.93 ms |
| 400+ | 0.40 ms / 0.96 ms |
| **Overall** | mean 0.21 ms, median 0.18 ms, **0/640 epochs exceed 1 ms** |

Roughly a **3000x** speedup over the `ceres::Covariance`-based approach (mean 687ms),
and critically the cost stays essentially flat as graph size grows 100→400+ blocks —
confirming it depends on residuals-per-epoch, not accumulated graph size, as
hypothesized. Total run wall time also dropped from 932s to 501s with this
diagnostic running every epoch, consistent with the per-call cost drop. **This
resolves the load-bearing feasibility question: a local, current-epoch cross-
information computation is real-time-viable.**

**Open issue found in the same measurement, to resolve before designing the AR
mechanism on top of this:** 285/640 epochs (44.5%) have a near-zero minimum eigenvalue
in the local information matrix (i.e. it's rank-deficient/near-singular on its own),
and 240 of those show the *exact same* floating-point value (0.000278) across epochs
with different graph sizes and ambiguity counts — too consistent to be numerical
coincidence, more likely a specific direction (probably an ambiguity or attitude
component) that genuinely has zero local information from the current epoch's
residuals alone (e.g. an ambiguity just added this epoch with no phaserange
observation yet, or an attitude component that needs multiple epochs of IMU
integration to become observable). This is an expected consequence of using
single-epoch information in isolation, not necessarily a bug — but means the final
design cannot use this local information standalone; it must be *fused with a prior*
(the existing shadow-estimator covariance, or the marginalization prior already
computed for the sliding window) before it's usable for AR, rather than treated as a
complete replacement.

Also noted: `AmbiguityResolution::addStableFixationToGraph()` is declared
(`include/gici/gnss/ambiguity_resolution.h:260`) but has no implementation and no call
site anywhere — dead code, apparently intended for constraining stable-fixed
ambiguities across epochs (not cross-sensor coupling as far as the header comment
suggests, but worth re-checking once the design is more concrete).

## Plan

1. ~~Measure the cost of a naive `ceres::Covariance`-based joint query.~~ **Done
   2026-07-17 — infeasible, see cost measurement result above.**
2. ~~Implement + measure the Jacobian-based local information approach.~~ **Done
   2026-07-17 — confirmed tractable (~3000x faster, cost independent of graph size).
   Also found: local-only information is near-singular in ~45% of epochs — must be
   fused with a prior, see above.**
3. **Fuse the local information with a prior** (current step): combine
   `getLocalCrossInformation()`'s output with the existing shadow-estimator's
   ambiguity covariance (information-form addition: convert the shadow covariance to
   information, add the local cross-information, invert the sum) so the result is
   well-conditioned even when the current epoch alone under-constrains some
   direction. This also has a nice framing: the shadow estimator already gives a
   correct-if-decoupled *ambiguity* prior; this adds the *cross* terms it structurally
   cannot have (no visual/IMU states in that shadow graph at all).
4. Design how the resulting fused cross-information actually improves AR —
   candidates: (a) reshape the MLAMBDA decorrelation transform with it instead of the
   GNSS-marginal covariance, (b) tighten/adapt the ratio-test threshold based on the
   fused joint uncertainty, (c) a visual-consistency check on candidate fixed
   solutions (closer to Ran et al. 2026's mechanism — prefer (a) or (b) if possible,
   to stay differentiated from the external-signal literature).
5. Implement as an opt-in estimator option (do not change existing locked baselines'
   default behavior) so `verify_upstream_fidelity.sh`-style comparisons stay valid.
6. Evaluate on the existing locked baselines (GICI-board 1.1/3.1/4.1, UrbanNav
   Medium/Deep) — Medium (100% float today) is the primary test case: does fixed-rate
   or accuracy improve without regressing the others?
7. Ablation experiments folded into the same paper (not separate submissions, per
   2026-07-17 discussion): base-RINEX-interval robustness (from the Medium fix) and
   resource-aware real-time behavior (from `research/ros2-live-nosparsify-experiment`)
   as supporting robustness results for the same core contribution, not separate claims.
8. Deeper literature pass before submission (the 2026-07-17 pass above was a first
   scoping check, not exhaustive) — particularly around the GS-GVINS / PO-GVINS /
   TITS 2024 factor-graph-AR line of work, to confirm none of them already do the
   current-epoch local-information approach.

## First end-to-end result (2026-07-17), THEN corrected — see next two sections

The first working version fused the shadow-estimator covariance with the local
cross-information by **adding** the shadow's ambiguity information on top of the local
ambiguity block (`fused_aa = L_aa + S`). It gave a large apparent Medium improvement
(8.04m → ~2.92m mean of 2 runs) but an adversarial code review (below) then showed the
number was partly a **statistical artifact** (double-counting). The two sections that
follow supersede it. The raw first-version numbers, for the record: Medium 2.564 /
3.275 m (mean 2.92 m / 1.44°); GICI-board 1.1 0.0291m/0.485°, 4.1 0.0708m/0.722° (no
regression); **3.1 0.1533m/1.300° — a rotation regression** (Δ1.300−1.078 = 0.222° >
0.15° tol); UrbanNav Deep — could not evaluate (pre-existing crash, see below).

## Adversarial code review → double-counting bug found and fixed (2026-07-17)

An independent adversarial review (5 dimensions, 2-vote verification) plus a separate
targeted audit both converged on a real statistical defect in `fused_aa = L_aa + S`:
the shadow estimator's 2-epoch window and the local matrix's ambiguity block `L_aa`
**both contain the current epoch's GNSS DD phase/code information** (same physical
observations, re-instantiated in two graphs), so the shared measurement was counted
twice → the fused ambiguity covariance was systematically too small (overconfident) →
the ratio test accepted marginal (possibly wrong) fixes more readily than the true
statistics justify. This is the leading mechanism for the 3.1 rotation regression.

**Fix (single-use local information).** `estimateVisionAidedAmbiguityCovariance()` now
uses the local joint information *alone* — each current-epoch measurement used exactly
once, no shadow term added — and marginalizes pose/speed-and-bias out of it (full-
matrix inverse + top-left block). When the current epoch under-constrains a direction
(~45% of epochs), it returns false and the caller falls back to the plain shadow
covariance rather than fabricating confidence. Re-evaluated (author `evo_ape` / GICI
board author eval):

| Dataset | Locked baseline | First version (double-count) | **Single-use fix** |
|---|---|---|---|
| Medium (evo_ape) | 8.042 m / 2.108° | ~2.92 m / 1.44° | **3.211 m / 1.533°** |
| GICI 3.1 | 0.142 m / 1.078° | 0.153 m / **1.300° ✗** | **0.138 m / 1.249°** |
| GICI 4.1 | 0.073 m / 0.696° | 0.071 m / 0.722° | **0.067 m / 0.756°** |
| GICI 1.1 | 0.0292 m / 0.471° | 0.0291 m / 0.485° | **0.0291 m / 0.480°** |

Key conclusions: (1) the Medium improvement **survives** the fix (3.211m still beats
the paper's 3.40m and is within the first version's own run-to-run spread), so it was a
**real** effect, not purely a double-count artifact; (2) translation *improved* on
every GICI-board dataset; (3) the 3.1 rotation regression **shrank** (1.300°→1.249°)
but a marginal residual (Δ0.171° > 0.15°) remained — attributable to either the still-
present conditioning approximation, or run-to-run variance. **UrbanNav Deep** still
cannot be evaluated: it crashes (`Check failed: seq.size()>1`, `common.h:114`) after a
total satellite dropout; a causal test (option OFF → plain baseline path) crashes
identically, proving this is a **pre-existing GICI-LIB robustness gap, unrelated to this
contribution** (root cause: relative-residual helpers assume ≥2 history entries; a total
dropout erases the deque below that — not part of this branch's scope to fix).

## Genuinely vision-aided AR — a new estimator type `rtk_imu_camera_rrr_va` (2026-07-17)

The review also established that in the single-use fix, **visual information never
enters directly**: AR runs only at GNSS epochs (gPose states), and reprojection
residuals attach only to camera keyframe states (cPose), so vision reached the
ambiguity covariance only indirectly through the IMU chain — the mechanism was really
*IMU/pose-coupling-aided*, not *vision-aided*. To make the claim honest, a new
first-class estimator type was added (leaving the original estimator and the frozen
baselines byte-untouched):

- **`EstimatorType::RtkImuCameraRrrVa`** / yaml `type: rtk_imu_camera_rrr_va` —
  `RtkImuCameraRrrVaEstimator` (`include/gici/fusion/rtk_imu_camera_rrr_va_estimator.h`
  / `.cpp`), a subclass of `RtkImuCameraRrrEstimator` overriding only the AR-covariance
  method. It extends the local cross-information parameter set with the chain of states
  from the current GNSS pose back to (and including) the most recent camera keyframe
  (cPose), kept connected through the intervening IMU pre-integration factors. Because
  reprojection residuals attach to that cPose, **genuine visual information now enters
  the joint ambiguity covariance.** Verified: `num_camera_keyframes=1` in ~1050/1140
  post-init epochs on GICI-board 1.1 (vs 0 before init), i.e. vision is actually in the
  matrix, not just nominally.

- **Crash found and fixed during bring-up.** Including the cPose chain first triggered a
  non-deterministic heap crash. Bisection (minimal override = 1775 clean epochs) placed
  it in `getLocalCrossInformation` when the chain reached back far enough to pull in the
  **marginalization prior** — a dynamically-sized residual that the original dead-code
  Jacobian pattern (`getLhs`) never exercised. Fix: `getLocalCrossInformation` now skips
  `ErrorType::kMarginalizationError`. This is both a crash fix and *more correct* — the
  marginalization prior summarizes already-marginalized PAST states, so folding it into
  a "current-epoch local information" matrix would conflate accumulated history with the
  current measurement. Confirmed clean under **AddressSanitizer** (1137 epochs, 0 heap
  errors). Approach-A callers (current pose only) never reached the prior, so their
  results are unchanged.

**Superseded single-run results — genuine vision (`rtk_imu_camera_rrr_va`):**

| Dataset | Locked baseline | Single-use (IMU/pose only) | **Genuine vision (va)** | Paper |
|---|---|---|---|---|
| **Medium** (evo_ape) | 8.042 m / 2.108° | 3.211 m / 1.533° | **2.201 m / 1.478°** | 3.40 m / 1.30° |
| GICI 3.1 | 0.142 m / 1.078° | 0.138 m / 1.249° | **0.157 m / 1.219°** (pass) | 0.29 / 1.58 |
| GICI 4.1 | 0.073 m / 0.696° | 0.067 m / 0.756° | **0.070 m / 0.748°** (pass) | 0.08 / 0.54 |
| GICI 1.1 | 0.0292 m / 0.471° | 0.0291 m / 0.480° | **0.0291 m / 0.506°** (pass) | 0.03 / 0.54 |

At the time, Medium looked promising (2.201 m vs 8.04 m locked baseline), and the
GICI-board single runs were within the locked tolerances. Do **not** use this as a
positive claim: the later repeated Deep evaluation below supersedes this reading and
shows the same mechanism can degrade horizontal accuracy despite increasing fixed-rate.

**Caveats (single runs — must be firmed up before any claim):** Medium has a known
~0.7 m run-to-run spread (Ceres `num_threads` non-associativity). The va−vs−single-use
Medium gap (2.20 vs 3.21 m ≈ 1 m) exceeds that spread, which is suggestive but not
conclusive from one run each — a repeat-run variance characterization (3 va + 3
single-use Medium runs) is in progress. No epoch-level false-fix validation yet (only
aggregate accuracy). The conditioning approximation (fix neighbors, don't marginalize
them) is still present in the va estimator.

## False-fix validation (2026-07-17) — the make-or-break AR-safety test

Aggregate APE left an ambiguity: fix rate rose significantly (5.3%→7.1%) but 3D
evo_ape did not clearly improve. That pattern *could* mean the extra fixes are false
(wrong-integer) fixes. Tested directly with a per-epoch, offset-robust check
(`scripts/.../false_fix_validate.py`): horizontal ENU error vs interpolated GT, split
by GPGGA fix status, after removing a slowly-varying common offset (sliding-window
median) so only high-frequency deviation remains — a false fix shows up as a
wavelength-scale (dm) jump that offset removal does NOT absorb.

Result, **robust across all 6 Medium runs** (va-full ×3, reprojection-ablation, IMU-only ×2):

| | FIXED epochs (median / p95 / max) | FLOAT epochs (median) | median ratio | fixed epochs > 0.30 m |
|---|---|---|---|---|
| every run | ~0.01 m / ≤0.14 m / ≤0.25 m | ~0.07–0.08 m | 0.11–0.25 | **0.0%** |

**Verdict from this check only:** no wavelength-scale high-frequency jumps were found.
This does **not** prove globally correct fixing. The Deep reassessment below showed the
check is blind to slow drift-inducing wrong/overconfident fixes, so it is necessary
but insufficient.

Two honest caveats:
- **Correctness is not vision-specific.** va-full, the reprojection-ablation, and plain
  IMU-only ALL fix correctly at cm level — so "correct fixing" is a property of the
  joint-graph local-information method broadly, not of the visual term specifically.
- **Cross-run "do the extra fixes help accuracy?" is inconclusive by construction.**
  A correct fix corrects a slowly-varying integer bias, which the sliding-median offset
  removal absorbs; so the offset-removed metric cannot see the aggregate fixing benefit
  (it is valid only for detecting false-fix jumps, which it did). Demonstrating an
  aggregate accuracy gain needs a scenario where fixing — not slow drift — dominates the
  error budget; Medium is drift-confounded (and Deep, the natural second urban test,
  crashes on a pre-existing bug).

## Pre-existing Deep-crash fix + Deep evaluation (2026-07-17) — mixed, then NEGATIVE on accuracy

> **Bottom line (after n=3 repeats — supersedes the single-run optimism below):** the va
> estimator makes ~13× more fixes that pass the high-frequency false-fix check, but its
> **overall horizontal accuracy is consistently WORSE than baseline on Deep** (rmse_h
> 6.75±2.98 m vs 2.77±0.44 m, every va run worse than every baseline run) and neutral on
> Medium. Increasing the fix rate does **not** improve positioning here — on the harsh
> dataset it degrades it. This is a **negative result** for the accuracy claim. See "n=3
> reassessment" at the end of this section.

**Crash fix.** The Deep total-GNSS-outage crash was root-caused (gdb): `getLast()` on the
GNSS history deques (`gnss_measurement_pairs_`, `ambiguity_states_`) inside
`addGnssMeasurementAndState`. The pre-existing guards (`isFirstEpoch()`,
`lastGnssState().valid()`) were the wrong proxies — `isFirstEpoch()` checks
`states_.size()`, which counts camera keyframes too, while `getLast()` needs ≥2 GNSS
epochs. During Deep's total outage the GNSS deque shrinks to one entry while the guards
still pass → `CHECK(seq.size()>1)` aborted. Fixed by guarding both call sites on the
actual GNSS-deque size (`rtk_imu_camera_rrr_estimator.cpp`, cycle-slip + relative-error
blocks). Skipping cycle-slip/relative constraints with <2 GNSS epochs of history is also
physically correct (a total outage is a full cycle slip). **Frozen baseline preserved:**
GICI-board 1.1 plain still passes vs locked (0.0294m/0.486° vs 0.0292m/0.471°); the fix
only affects the outage path that previously crashed.

**Deep single-run (what first looked like a strong result — DO NOT cite without the n=3
reassessment below):** va 2.503 m / 0.727° / 19.8% fixed vs baseline 2.341 m / 1.128° /
1.3% fixed; fix rate and rotation looked monotonic in the vision components (baseline →
ablate → va) and va's 2990 fixes all passed the offset-removed false-fix check (median
1.4 cm, 0% > 0.30 m). This single run motivated an (incorrect) "strong result" reading.

### n=3 reassessment (2026-07-17) — the single run was not representative

Deep, 3 runs each of va and baseline (identical crash-fixed binary, differing only in the
estimator), author eval + horizontal RMSE (`rmse_h`, the cleaner metric — 3D evo_ape is
vertical-bias-dominated here):

| Metric | va (genuine vision) | baseline (plain) |
|---|---|---|
| **rmse_h horizontal (m)** | **6.75 ± 2.98**  [4.37, 4.92, 10.95] | **2.77 ± 0.44**  [2.68, 3.33, 2.28] |
| evo_ape 3D pos (m) | 3.82 ± 1.75 | 2.35 ± 0.25 |
| evo_ape rotation (°) | 0.89 ± 0.12 | 0.99 ± 0.10 |
| fixed_rate (%) | 17.0 ± 1.9 | 1.26 ± 0.05 |

- **Robust POSITIVE:** va fixes ~13× more ambiguities (17% vs 1.3%), stable across runs,
  and those fixes pass the high-frequency false-fix check.
- **Robust NEGATIVE:** va's horizontal accuracy is **worse than baseline on every run**
  (best va 4.37 m > worst baseline 3.33 m). Baseline rmse_h ~2.77 m matches the locked
  reference (2.55 m), so baseline is correct and va is genuinely degraded. Rotation is
  marginally better on average (0.89 vs 0.99°) but the ranges overlap — not a clean win.
  The single-run 0.727° was a favorable va run against an unfavorable baseline run.

**Why more (high-frequency-correct) fixes yield WORSE accuracy — and the flaw in the
earlier false-fix check.** The false-fix validation detects wavelength-scale *jumps* but
is blind to *drift-inducing subtly-wrong fixes*: a wrong integer shifts position by a
fraction of a wavelength (dm), appearing as slow drift rather than a jump, and the
sliding-median offset removal absorbs exactly that. So "0% false fixes" was **necessary
but not sufficient**. The rmse_h degradation indicates va's aggressive extra fixing
injects slow drift — the fixes are locally consistent (cm high-frequency) but not
globally correct. Equivalently, the local-cross-information covariance is overconfident
(the conditioning approximation), so AR accepts fixes that are locally plausible but
globally slightly wrong.

**Verdict: NEGATIVE result for the accuracy claim.** The contribution increases the
correct-fix *rate* but does not improve, and on the harsh dataset degrades, positioning
accuracy. It is not publishable as a positive "vision-aided AR improves accuracy" result.
What survives honestly: (1) the tractable local-cross-information *method* itself (a real,
~3000× cheaper way to get partial joint covariance); (2) the pre-existing total-outage
crash fix; (3) a cautionary empirical finding — *raising RTK fix rate via an overconfident
joint-graph covariance can degrade accuracy through drift-inducing fixes that evade
high-frequency false-fix detection* — which is itself a legitimate (negative/methodological)
contribution, and implies the real open problem is a *consistent* (non-overconfident)
joint covariance, i.e. proper marginalization of the neighbor states rather than the
conditioning approximation.

## Exact full-graph covariance path (2026-07-17) — formula fix, single-run evaluation

The VA estimator now has a clean covariance path enabled by
`ar_use_exact_joint_covariance: true` in
`research/config/rtk_imu_camera_rrr_va_urbannav.yaml`. Instead of the previous

```text
I_final = I_shadow + (A_with_reprojection - A_without_reprojection)
```

it asks the real active Ceres graph for the covariance of only the current ambiguity
blocks:

```text
Q_aa = Cov_graph(a, a) = (H^{-1})_aa
     = (H_aa - H_an H_nn^{-1} H_na)^{-1}
```

where `a` are the ambiguity blocks and `n` is every other active nuisance block in the
same graph: poses, speed/bias, camera keyframes, visual landmarks, extrinsics,
clock/frequency states, relative constraints, and marginalization-prior variables.
This is the statistically clean Schur-complement treatment for the current
linearization/window: no GNSS shadow term is added, no reprojection delta is added, and
unrequested neighboring states are not held fixed by the VA formula.

Tradeoff: this reintroduces the cost the original authors avoided with their shadow
estimator. Earlier measurements showed `ceres::Covariance` scales with full graph size
(hundreds of milliseconds per epoch once the window fills). The exact path is therefore
the correctness reference, not a real-time claim.

**Single-run exact-config evaluation (2026-07-17/18).** Ran the full Medium and Deep
wrappers with `use_vision_aided_ambiguity_resolution=true`,
`ar_use_exact_joint_covariance=true`, `ar_use_delta_information=false` into
`results/research/urbannav_va_exact_20260717_233807`:

| Dataset | Horizontal RMSE | Yaw RMSE | Fixed / float / single | Matched | Wall time | Paper ref |
|---|---:|---:|---:|---:|---:|---:|
| Medium | 2.618 m | 1.881 deg | 0.00% / 100.00% / 0.00% | 7379 | 1133 s | 3.40 m / 1.30 deg |
| Deep | 2.647 m | 0.634 deg | 1.46% / 98.45% / 0.09% | 15119 | 3149 s | 2.46 m / 1.64 deg |

Interpretation before the rank-deficiency fix: these were **exact-config** results, not
a clean proof that exact covariance was available at every AR epoch. The Ceres
covariance backend frequently reported rank-deficient full problems and returned
failure (`Failed to compute
covariance!`): 471 times on Medium and 1198 times on Deep. The VA covariance override
therefore returned false on those epochs and the caller fell back to the plain shadow
ambiguity covariance. This is statistically safer than fabricating a covariance, but it
meant that implementation was only a formula-correct reference path with frequent
fallback, not yet a robust exact-Schur AR engine.

**Gauge/rank-deficiency handling fix (2026-07-18).** `Graph::computeCovariance()` now
tries sparse QR first, then automatically falls back to Ceres `DENSE_SVD` with
`null_space_rank=-1` when the full graph is numerically rank deficient. This computes
the Moore-Penrose covariance by dropping gauge/null-space modes instead of returning
false. It also extracts covariance blocks through a row-major buffer, matching Ceres'
documented memory layout, and validates finite output before returning success. A
Medium smoke run in `/tmp/gici_cov_svd_smoke` saw 26 sparse QR rank-deficiency events,
25 SVD recoveries before manual SIGINT, and no terminal SVD failure; the final event was
interrupted after QR failure and before recovery logging.

Current verdict: the VA-layer formula is correct (`Q_aa=(H^{-1})_aa`, equivalent to
eliminating all nuisance states by Schur complement), and the graph covariance backend
now has an explicit gauge/rank-deficient recovery path. A full Medium/Deep exact rerun
is still required to replace the pre-fix metrics above with post-fix metrics.

## Consistent + real-time marginal covariance — `Graph::getMarginalAmbiguityCovariance` (2026-07-18)

The n=3 negative result above identified the real open problem: a *consistent*
(non-overconfident) joint ambiguity covariance that is also *real-time*. This is now
implemented as a first-class method and wired as the VA estimator's default path
(`ar_use_fast_marginal_covariance: true`; precedence fast → exact → delta).

**Method.** Assemble the loss-corrected (Triggs/BANS Eq. 11, identical to Ceres'
`corrector.cc`) Gauss-Newton information of the ACTIVE sliding-window graph in minimal
coordinates — including the marginalization prior, folded in exactly as
`Lambda = J_^T J_` via a new read-only accessor
`MarginalizationError::marginalizationInformation()` (never through the generic
per-residual evaluation buffers, whose dynamic sizing caused the historical heap crash) —
partitioned `[a | n | l]`: ambiguities, dense nuisance (poses/speed-bias/clocks/
extrinsics/prior-touched blocks), and visual landmarks. Then eliminate exactly:

1. **Stage 1 — landmarks, block-by-block:** each 3×3 landmark information is
   rank-truncated at an eps-relative tolerance and Schur-eliminated locally. For a PSD
   Gauss-Newton sum, a null mode of `H_ll` has identically zero coupling rows, so the
   Moore-Penrose drop IS the exact marginalization of that mode. This isolates
   weak-parallax landmarks so they cannot poison the global conditioning (measured to be
   the dominant failure mode of one-shot factorization: `SimplicialLDLT` reported success
   but returned solutions inflated ~1e8 on ~5% of epochs).
2. **Stage 2 — dense nuisance:** Jacobi-equilibrated (unit-scale-free, the same
   preconditioner pattern as `MarginalizationError::updateErrorComputation`) dense LDLT
   with **iterative refinement as self-validating arithmetic**: the last refinement
   increment bounds the remaining linear-algebra error, propagated to the Schur
   complement and enforced at the final gate — `Q_aa` is returned only when that bound is
   provably below `lambda_min(S)`, i.e. when arithmetic error cannot materially change the
   covariance. Otherwise the routine returns false with a machine-readable reason and the
   caller falls back to the plain shadow covariance. No jitter, no tuned constants; every
   threshold is machine-epsilon-derived or a convergence criterion.

Since Schur complements compose and `H_al = 0` structurally (no residual connects an
ambiguity to a landmark; prior-touched landmarks are promoted to the dense partition),
the result equals `[H_active^{-1}]_aa` — the same quantity `ceres::Covariance` computes —
at a cost bounded by the window, not the trajectory.

**Runtime equivalence + cost validation (GICI-board 1.1, full trajectory, per-AR-epoch
benchmark `[vaar-fast]`, fast vs whole-problem `ceres::Covariance` on the same live
graphs, 1787 AR epochs):**

| Metric | Result |
|---|---|
| fast_ok (usable covariance produced) | 99.94% (1 abstention / 1787) |
| rel_err vs ceres ≤ 1e-3 | 90.1% of epochs (59.6% ≤ 1e-4) |
| disagreements > 1e-3 | 177 (9.9%) — conservative (larger trace) 171; trace below ceres 6, worst deficit **7.3e-4** at the 1e-3 boundary (numerical ties, not material overconfidence) [corrected 2026-07-19 by scripts/eval_paper_tables.py; earlier "all 177 conservative / 0 overconfident" was an over-round of the trace proxy] |
| fast cost | mean 28.0 ms, max 97.5 ms (bounded by window) |
| ceres cost (same epochs) | mean 811.9 ms, max 2714 ms (grows with graph) |
| mean speedup | ~29x, and worst-case 2.7 s -> <0.1 s |

Critically, the **direction check**: on 171/177 disagreeing epochs the fast covariance
trace was LARGER than the ceres reference; the other 6 sit within 7.3e-4 (trace-relative) at
the 1e-3 boundary — numerical ties, never material overconfidence. Interpretation: those windows
contain near-unobservable ambiguity directions whose tiny reduced eigenvalues the
refinement gate proves are resolved (not noise); ceres' sparse QR silently rank-truncates
them and reports confidence that is not there, while this method reports the honest large
variance — the AR-safe direction (LAMBDA simply declines to fix such an ambiguity). The
method is therefore *never overconfident relative to the reference* on measured data —
exactly the property whose absence caused the Deep accuracy regression.

**Frozen-baseline preservation (post-change binary):** plain `rtk_imu_camera_rrr` on 1.1:
0.028927 m / 0.483° vs locked 0.029198 m / 0.470557° — within tolerance (0.005 m / 0.1°),
1775/1775 epochs.

**VA no-regression on 1.1 (open-sky):** `rtk_imu_camera_rrr_va` with the fast marginal
covariance: **0.028991 m / 0.4727°** — inside the locked-baseline tolerance band, with
fix rate 869/1775 fixed (49.0%) vs baseline 876/1775 (49.4%) — statistically identical.
As expected on open-sky where AR already works: the consistent covariance neither
inflates the fix rate (the overconfident failure mode) nor loses fixes.

**Evaluation-integrity audit (2026-07-18, independent):** pipeline confirmed leak-free —
no GT reaches the estimator; baseline and VA configs byte-identical except the algorithm
flags; the head-to-head metric is raw ENU RMSE with no alignment fitted on scored data;
research outputs are directory-isolated from locked baselines. Two hardening fixes from
the audit: `run_author_eval_urbannav.sh` now reads the `algorithm` label from the
producing config instead of hardcoding the baseline name; and note that
`false_fix_validate.py` (cited in the false-fix section above) is not present in the
working tree — those tables are historical and not currently reproducible from the repo;
re-commit the script before relying on them in a paper. `urbannav_detect_eval_start.py`
(GT-based trim helper) remains DIAGNOSTIC ONLY and is invoked by no scoring path — it
must stay out of evaluation.

**Status / claims discipline:**
- Proven here: numerical equivalence-or-conservative vs the exact joint covariance on
  live graphs; real-time cost (bounded by window); frozen baselines preserved; leak-free
  evaluation; **ASan-clean** (fresh instrumented build, 1761 GICI-board 1.1 epochs through
  the full fast path including the marginalization-prior accessor: 0 heap errors).
- **UrbanNav n=3 evaluation (2026-07-18, data restored; see section below):** safety
  condition PASSED (no catastrophic degradation); vertical + yaw improve consistently on
  Deep (~35%/~40%, 3/3 runs) with 5x fix rate; horizontal slightly better on Medium (3/3)
  and statistically unresolved on Deep (worse 2/3, better 1/3, overlapping spreads).
  Remaining false-fix validation requires rebuilding the missing script.

## UrbanNav n=3 evaluation of the consistent covariance (2026-07-18)

Data restored (Medium symlinked from Downloads; Deep on Data1). 12 runs: n=3 x
{Medium, Deep} x {baseline, VA-fast}, same binary, byte-identical configs except the
estimator type, VA/baseline pairs executed in parallel per run. Baselines were RE-RUN
into `results/research/urbannav_baseline_recheck_20260718/` (locked artifacts untouched).
Metrics: the runner's own raw-ENU `evaluate_solution` (no alignment), evaluated post-hoc
because gici_main hangs on SIGINT after solution completion and the runner marks the
run FAILED despite a complete solution (all 12 runs produced full 7379/15119-epoch
solutions; runner `ok_exits` should later accept the watchdog-stable SIGKILL path).

| Metric (mean +- sd, [runs]) | VA-fast | Baseline | Pairwise delta (VA-BL) |
|---|---|---|---|
| **Medium** rmse_h (m) | **2.845 +- 0.100** | 2.986 +- 0.163 | -0.211 / -0.139 / -0.069 (3/3 better) |
| Medium rmse_u (m) | 14.14 +- 1.40 | 21.64 +- 12.06 | better 2/3 (BL has a 35.6 outlier) |
| Medium yaw (deg) | 1.907 | 1.916 | ~tied |
| Medium fixed rate | 0.0% | 0.0% | consistent covariance does NOT fabricate fixes |
| **Deep** rmse_h (m) | 3.401 +- 0.784 | 3.026 +- 0.679 | +0.86 / +0.56 / **-0.30** (mixed) |
| **Deep** rmse_u (m) | **1.463 +- 0.213** | 2.266 +- 0.267 | -1.09 / -1.00 / -0.32 (**3/3 better, ~35%**) |
| **Deep** yaw (deg) | **0.657 +- 0.073** | 1.089 +- 0.279 | -0.06 / -0.75 / -0.49 (**3/3 better, ~40%**) |
| Deep fixed rate | **8.8 +- 1.2%** | 1.7 +- 0.1% | ~5x more fixes |

**Verdict vs the pre-registered criteria:**
1. **Safety: PASS.** The old overconfident variant was worse on every Deep run by huge
   margins (6.75 +- 2.98 vs 2.77 +- 0.44). The consistent covariance shows no such
   mechanism: Deep horizontal spreads overlap (VA 2.73-4.27 vs BL 2.34-3.70) while the
   fix rate still rises 5x — the drift-inducing-wrong-fix failure mode is gone.
2. **Positive accuracy result: PARTIAL.** Consistent wins: Medium horizontal (3/3,
   modest), Deep vertical (~35%, 3/3), Deep yaw (~40%, 3/3), Deep fix rate (5x). NOT
   resolved: Deep horizontal (mean +0.375 m = +12%, but sign flips across runs and
   spreads overlap — cannot claim improvement, cannot claim significant degradation
   at n=3).
3. Honest interpretation: correct fixes constrain exactly the directions float RTK/INS
   is weakest in (vertical, yaw) — that is where the gain shows first. The unresolved
   Deep horizontal delta points at the **decision layer**, which is still stock: the
   ratio-test threshold (2.0) was tuned for the shadow covariance, the post-fix check is
   GNSS-range-only, and partial-AR subset selection predates the consistent covariance.
   Those are the deliberately-deferred next steps (adaptive/FFRT acceptance; joint
   vision+IMU+GNSS post-fix validation), now motivated by data rather than speculation.
4. To fully resolve Deep horizontal: n>3 repeats and/or the decision-layer upgrade, plus
   rebuilding the false-fix analysis (script missing from tree) to classify the extra
   VA fixes directly.

## Paired per-epoch diagnosis + decision-layer upgrade "VA-v2" (2026-07-18)

**Diagnosis (rebuilt false-fix analysis, paired per-epoch, all 6 Deep runs).** For each
(run, epoch) matched between VA and baseline: delta horizontal error conditioned on VA
fix status, contiguous-fixed-segment analysis (before/during/after), and the legacy
sliding-median high-frequency jump check:

| Deep | dh at FIXED epochs (median) | dh at FLOAT epochs (median) | segment before/during/after | HF>0.3m at fixes |
|---|---|---|---|---|
| run1 (VA worse) | ~+0.02 m | **+0.71 m** | 0.04 / 0.02 / 0.02 | 2.1% |
| run2 (VA worst) | +0.05 m | **+1.29 m** | 0.06 / 0.22 / **+0.69** | 2.4% |
| run3 (VA better) | -0.003 m | -0.06 m | ~0 | 1.4% |

**Mechanism established:** the extra fixes are NOT locally wrong (2-5 cm at the fixed
epochs themselves; HF check clean) — the harm lives in the FLOAT epochs afterwards
(+0.7-1.3 m in the two bad runs, in run2 largest right AFTER fixed segments). I.e.
marginal fixes, locally consistent, get marginalized into the prior and bias the
subsequent float trajectory — the same drift-injection mechanism as the old
overconfident variant, but ~5x weaker under the consistent covariance and present in
only 2/3 runs.

**Decision-layer upgrade (both opt-in via AmbiguityResolutionOptions, defaults preserve
upstream byte-identically; enabled only in the VA UrbanNav config):**
1. `use_joint_cost_validation` — vision/IMU veto at acceptance: extends the upstream
   zero-threshold rule (reject if GNSS range cost increases after the constrained
   1-iteration re-solve) to also reject if the robustified reprojection+IMU cost
   increases. No new constants (exact mirror of the upstream rule).
2. `min_bootstrap_success_rate: 0.999` — integer-bootstrapping success-rate gate on the
   LAMBDA-decorrelated conditional variances (Teunissen P_s; LD + reduction faithfully
   ported from vendored RTKLIB lambda.c), applied per partial-AR subset like the existing
   per-ambiguity std gate. The 0.1% failure budget is a pre-registered probabilistic
   specification, not a GT-fitted constant.

**Validation so far:** frozen baseline 1.1 re-verified after the AR-code edits with gates
OFF (0.028968 m / 0.470380 deg vs locked 0.029198/0.470557 — pass); smoke run on 1.1 with
gates ON: no crash, open-sky fixing intact, no over-veto.

**Experimental-protocol pitfall found and corrected (2026-07-18).** Post-file replay is
NOT wall-clock-paced at all: `FilesReading::run()` (`src/stream/files_reading.cpp`) is a
sleep-free spin loop that pushes data as fast as the downstream callbacks accept it
(`replay: speed` only applies to time-tagged `type: file` streams, not `post-file`).
Wall time therefore equals estimator consumption speed — measured spans for the same
25.2-min Deep sequence ranged 14–24 min across runs with identical work done (~7450
keyframes each). Because `max_solver_time: 0.04` is a wall-clock budget, CPU
contention/frequency changes the effective solver iterations per epoch, so absolute run
results are load-sensitive. The matrix-1 protocol was valid because each wave ran the VA and
baseline SIMULTANEOUSLY (identical contention within the pair -> fair paired
comparison). The first VA-v2 matrix broke this by pairing VA-with-VA (and one solo run),
making its Deep numbers incomparable to the matrix-1 pairs (its "run2" slot produced an
outlier 8.0 m / 3.9 deg). Rule for all future runs: **within-pair simultaneous execution,
one pair at a time; never compare runs across different contention regimes.** The Deep
VA-v2 evaluation was re-run under the correct paired protocol
(`urbannav_va_v2_paired_20260718` + fresh paired baselines); Medium VA-v2 numbers
(2.63/2.80/... rmse_h, 0% fixed — gates never engage without fix attempts) are unaffected
by pairing because both VA variants reduce to the same float estimator there.

## Final paired VA-v2 Deep evaluation + honest verdict (2026-07-18)

**Protocol.** 3 sequential waves; each wave ran VA-v2 and a FRESH baseline simultaneously
(identical contention within the pair), `urbannav_va_v2_paired_20260718` vs
`urbannav_bl_paired_v2_20260718`. All 6 runs complete (n_matched = 15119 each). Evaluated
with the runner's own metric (raw ENU vs GT interpolated to solution timestamps, no
alignment). Aggregates in `results/research/final_deep_3way_20260718.json`.

| Deep, n=3 paired | rmse_h (m) | rmse_u (m) | yaw (deg) | fixed |
|---|---|---|---|---|
| Baseline (fresh, paired) | 3.031 ± 0.439 | 2.871 ± 0.613 | 1.743 ± 0.903 | 1.3% |
| **VA-v2** | 4.614 ± 2.141 | **1.780 ± 0.211** | **0.783 ± 0.045** | 4.1% |
| Baseline (matrix-1, ref) | 3.026 ± 0.679 | 2.266 ± 0.267 | 1.089 ± 0.279 | 1.7% |
| VA-v1 (matrix-1, ref) | 3.401 ± 0.784 | 1.463 ± 0.213 | 0.657 ± 0.073 | 8.8% |

Per-wave pairwise deltas (VA − BL, negative = VA better):

| wave | d_h | d_u | d_yaw |
|---|---|---|---|
| v2 run1 | +0.43 | −1.64 | −1.55 |
| v2 run2 | **+3.65** | −1.12 | −1.37 |
| v2 run3 | +0.67 | −0.51 | +0.03 |
| (v1 run1..3) | +0.86 / +0.56 / −0.30 | −1.09 / −1.00 / −0.32 | −0.06 / −0.75 / −0.49 |

Across all 6 valid paired waves (v1 + v2): **vertical better 6/6** (sign test p ≈ 0.016),
**yaw better 5/6**, **horizontal worse 5/6** (mean +0.98 m, median +0.62 m, worst +3.65 m).
Baseline horizontal reproduces almost exactly across protocols (3.031 vs 3.026) — the
paired design is sound.

**Noise-floor calibration from Medium.** VA-v2 Medium n=3: rmse_h 2.628/2.800/2.926
(mean 2.785), fixed 0% in every run — and baseline Medium also fixes 0%. With zero
accepted fixes on both sides the two estimators are LOGICALLY IDENTICAL on Medium, so
the observed paired deltas there (~0.15–0.2 m, previously read as "VA better 3/3" in
matrix-1) measure the protocol noise floor (thread-timing/load nondeterminism), not an
algorithmic effect. This honestly reframes the Medium result as *parity*, and calibrates
Deep: |d| ≲ 0.2 m is noise; the 6/6 vertical (−0.5..−1.6 m) and 5/6 yaw (−0.5..−1.5°)
gains are far above it; the horizontal +0.43/+0.67 are marginal and +3.65 is a tail
event.

**Where the horizontal harm actually lives (paired per-epoch diagnosis of v2 waves).**
At VA-FIXED epochs VA-v2 is *better or equal* than its paired baseline (median dh
−0.03 / −0.01 / **−1.01** m per wave; du also ≤ 0) — the accepted integers are locally
GOOD. The harm is a heavy tail at FLOAT epochs: in the blow-up wave (run2, VA 7.08 vs BL
3.43) VA p50 = 2.77 vs BL 1.62 but p99 = 27.96 vs 9.59; the two catastrophic segments
(t = 528–630 s, VA ≤ 28.2 m where BL also degraded to 13.7 m; t = 1492–1512 s, VA ≤ 47 m
where BL stayed at 1.0 m) both occur ≥ 4–20 MINUTES after the last accepted fix (all
run2 fixes lie before t = 267 s). A direct "wrong integer → local jump" mechanism is
excluded; the only causal channel left is the everlasting marginalization prior plus
chaotic sensitivity of the deep-urban error process: extra constraints (even locally
beneficial ones) change the trajectory realization, and Deep's heavy-tailed multipath
turns realization changes into occasional large excursions. More intervention → more
rmse_h variance, even when every intervention is locally sound.

**Decision-layer postmortem.** The joint vision/IMU veto fired **0 times in every run**
(GICI-board and UrbanNav; the upstream range veto also 0) — consistent with the
diagnosis: accepted fixes do not bend the trajectory at acceptance time, so a
cost-at-acceptance check cannot see harm that materializes minutes later through the
prior. The P_s ≥ 0.999 gate halved the fix rate (8.8% → 4.1%) while keeping the
vertical/yaw gains — but did not remove the horizontal tail. The pre-registered
fallback rung (P_s = 0.99) is NOT triggered: its trigger was fix-rate collapse with
loss of gains, which did not happen. No further acceptance tuning will be done without
a new pre-registered hypothesis — iterating thresholds against GT outcomes would be
GT-tuning.

**Verdict (what is and is not supportable).**
- SUPPORTED, strong: the methodological core — consistent, real-time, exact-in-window
  marginal ambiguity covariance (per-epoch equivalence to `ceres::Covariance`, ~29×
  faster, zero overconfident disagreements, usable at 99.94% of AR epochs) with the
  frozen baseline untouched.
- SUPPORTED, robust across 6 paired waves: deep-urban **vertical** (−38..−50%) and
  **yaw** (−40..−60%) accuracy gains + 2.4–5× fix rate.
- NOT SUPPORTED: an across-the-board SOTA claim. Deep **horizontal** is worse in 5/6
  paired waves (median +0.6 m) with a heavy tail (+3.65 m worst), and the current
  decision layer (joint veto + P_s gate) does not remove it.
- Honest reframe: Medium "improvement" is protocol noise (0 fixes on both sides).
- The open mechanism is HOW fixes are applied, not WHICH fixes are accepted: upstream
  bakes each accepted fix into the everlasting marginalization prior via a hard
  Kalman-style constraint (1e-6 cycle² covariance). A principled next step is soft /
  revocable fix application (keep fixed-ambiguity factors as removable residual blocks
  inside the window; or down-weight the constraint by its own P_s), which attacks the
  spillover channel directly. That is a new research phase, properly pre-registered —
  not a threshold tweak.

## Decision-layer falsification chain (2026-07-18/19) — gates → expiry → dose → robust float

After VA-v2 established that a consistent covariance gives robust vertical/yaw/fix-rate
gains but a persistent deep-urban horizontal tail, we ran a pre-registered chain of
single-variable interventions, each with frozen decision bins (tail event := any run
d_h > +1.0 m vs baseline) and a GT-free mechanism check. Every intervention is opt-in;
the frozen upstream baseline is byte-identical with flags unset (re-verified per build).

**VA-v3 — revocable fixes (PREREG_SOFT_FIX.md).** Erase `kAmbiguityError` fix
constraints at marginalization instead of folding them into the everlasting prior
(`margin_ambiguity_fix_constraints: false`). Mechanism verified (21k+ constraints
erased). Two marginalizer-invariant bugs found and fixed en route (a residual-less
block trips the connectivity CHECK; a block the *detached* prior still references must
be marginalized through it, not removed — guard: erase only if no graph residuals AND
`!isParameterBlockConnected`). Outcome: catastrophic blow-ups gone, but the vertical
gain **inverted** and fix rate collapsed below baseline; confirmatory wave-1 tail
+1.24 m. **FAILURE of H1**, and the informative kind: benefit and harm share the same
channel — persistence of fix information in the prior. A binary keep/expire switch
cannot separate them.

**VA-v4 — decision-confidence-weighted fixes (PREREG_SOFT_WEIGHT.md).** Instead of a
switch, a dose: fix-constraint information = 1/[(1−P_s)·1 + P_s·1e-6] cycles⁻², the
Gaussian moment match of the integer decision mixture, P_s = bootstrapping success rate
of the exact accepted subset (`use_success_rate_fix_information`; no free parameters).
Stage-1 GICI-board 1.1: accuracy in the frozen band, fix count normal; open-sky P_s ≈ 1
→ upstream-identical (dose engages only where the model is uncertain). Mechanism check
PASS on Deep (44/445 accepted subsets softened, up to σ = 0.62 cycles). Design pitfall
caught and addendum'd: stacking the inherited P_s ≥ 0.999 gate on top of the dose
saturates P_s and neuters the mechanism — the gate was turned off so the dose is the
sole P_s consumer. Result, n=3 **solo** Deep (production-faithful, one run alone on an
idle machine):

| arm (solo Deep) | rmse_h (m) | rmse_u (m) | yaw (deg) | fixed |
|---|---|---|---|---|
| baseline | 3.230 / 2.508 / [run3 rerun] | 1.893 / 2.282 / — | 0.802 / 0.843 / — | 1.6% / 0.9% |
| VA-v4 (dose) | 3.06 / 3.65 / **5.23** | **0.894 / 0.887 / 1.234** | 0.642 / 0.765 / 0.685 | 6.1% / 5.9% / 7.0% |

Vertical is rock-stable and ~50% better than baseline; yaw better; fix rate ×3.8. But
the horizontal tail remains (run3 = 5.23 m ≫ BL mean + 1.0). **FAILURE of H2.** The
harm is invisible to every acceptance-time confidence measure (accepted subsets carry
P_s ≥ 0.999995 under the Gaussian model whose zero-mean assumption NLOS violates), so no
covariance-derived decision layer can gate or dose it away. Diagnosis note (run3
forensics): blow-ups in float-only stretches (t = 500–650 s mean|u→h| to 24 m; second at
900–1000 s), consistent with NLOS-biased float anchoring, not wrong integers. The
decision-layer ladder is declared **closed** in the prereg regardless of RF outcome.

**RF — bounded-influence float (PREREG_ROBUST_FLOAT.md).** The last in-estimator lever,
attacking the root cause directly: replace upstream's unbounded-influence
`HuberLoss(1.0)` on all GNSS residuals (pseudo/phase/doppler) with a redescending
`TukeyLoss(4.685)` (95%-efficiency constant, not tuned) so NLOS-biased measurements lose
influence on the *float* solution before AR ever runs (`use_bounded_influence_gnss_loss`;
`gnssLossFunction()` routes all 8 GNSS residual sites; `[rfloss]` mechanism log).
Confirmatory arm RF-VA (= VA-v4 + this flag) vs the frozen BL solo n=3; attribution arm
RF-BL (baseline + flag). A-priori caveat registered before runs: if the deep-urban bias
is carried by a *majority* of visible satellites (canyon geometry), no per-residual
M-estimator can help, and an H3 failure with a passing mechanism check specifically
implicates majority-bias geometry — pointing to external NLOS information (vision
classification, 3DMA) as the only remaining route.

*Result (2026-07-19, user protocol "one run first"):* frozen guard PASS (1.1 =
0.028942 m, baseline path byte-clean). RF-VA run1 **crashed at t ≈ 160 s** — the
`[rfloss]` mechanism log zeroed the majority or ALL GNSS residuals at 40/48 logged
epochs (25 zeroing 5/5; first event zeroed 3/3 at t=0), starving the float of its GNSS
anchor → divergence → `cv::findFundamentalMat` degenerate-point crash in visual init.
Cause: Tukey at c = 4.685 rejects residuals > 4.685 σ, but the deep-urban linearization
spread already exceeds that (a few-metre initial-position error at ~1 m pseudorange σ),
so the redescending kernel has no basin of attraction here — endemic, not merely
cold-start. **H3 (Tukey form) NOT runnable.** This directly confirms the a-priori
caveat: too large a fraction of residuals are simultaneously biased for any per-residual
weighting to isolate the bad ones — the deep-urban error is majority-affecting. Per user
decision, one exploratory **soft** variant follows (Cauchy c = 2.3849, retains a gradient
everywhere): RF-CVA runs crash-free and gently (down 2–3 of ~70 residuals, zeroed 0–2 —
basin preserved). run1 was tail-free (h=2.73, ATE Sim(3)=1.886 — best single Deep run of
the campaign), so user-directed it was promoted to a pre-registered n=3 confirmatory arm
(bins frozen 07:05, run1 disclosed as partial-unblinding). **VERDICT (07:35): FAILURE** —
h = 2.73 / **4.25** / 3.56 (mean 3.51 ± 0.76); run2 is a tail event (> BL+1.0). The
tail-free run1 was a lucky draw exactly as its n=1 / sub-noise-floor 0.14 m edge warned;
Cauchy is too gentle (2–3/70 residuals) to counter a majority-affected bias. u/yaw
retained (VA-v4 underneath). **The in-estimator campaign is CLOSED** — gates → expiry →
dose → Tukey-crash → Cauchy-tail all fail to remove the deep-urban horizontal tail. The
only remaining route is external NLOS scene information (vision classification / 3DMA) =
the vision-NLOS follow-on (research/VISION_NLOS.md). Pre-registration + n=3 discipline vindicated:
a single-run claim from RF-Cauchy run1 would have been wrong.

## Out of scope

- Changing any currently-locked baseline's default numbers (this is opt-in).
- The ROS2 real-time wrapper (this branch is file-mode / `research/standard-env`-rooted).
- HD-map-based approaches (would require new map data this repo doesn't have).

## Status

| Item | Status |
|---|---|
| Paper re-read for exact AR methodology + explicit gap quote | done (2026-07-17) |
| Empirical motivation from this repo's own baseline work | done — Medium's 100% float rate |
| Literature check (competitive landscape) | done, first pass — 2026-07-17; needs a deeper pass before submission |
| Codebase deep-dive of current AR implementation | done (2026-07-17) — gap confirmed, root cause (shadow-estimator workaround) found, entry point identified at `rtk_imu_camera_rrr_estimator.cpp:387` |
| Cost measurement of naive `ceres::Covariance` joint query | done (2026-07-17) — **infeasible**: 86.4% of epochs > 50ms, mean 687ms once the sliding window fills (vs shadow estimator's constant ~0.4ms) |
| Jacobian-based local information approach (`Graph::getLocalCrossInformation`) | done (2026-07-17) — **tractable**: mean 0.21ms, 0/640 epochs > 1ms, cost independent of graph size (~3000x faster than the naive approach) |
| Fuse local information with a prior (shadow-estimator covariance) | superseded — naive information addition double-counted current-epoch GNSS; delta-information VA path avoids that specific double count but remains overconfident because neighboring states are conditioned, not marginalized |
| Exact full-graph Schur covariance (`ar_use_exact_joint_covariance`) | implemented + pre-fix single-run Medium/Deep evaluated (2026-07-17/18); `Graph::computeCovariance()` now has sparse-QR -> dense-SVD gauge/rank-deficiency fallback (2026-07-18), smoke-verified, but full post-fix Medium/Deep metrics are still pending |
| **Fast consistent marginal covariance (`ar_use_fast_marginal_covariance`, default VA path)** | done (2026-07-18) — `Graph::getMarginalAmbiguityCovariance`: two-stage exact elimination (per-landmark rank-truncated Schur -> equilibrated dense LDLT with iterative-refinement self-validation), prior included via `MarginalizationError::marginalizationInformation`; validated per-epoch vs `ceres::Covariance` on full 1.1 (1787 AR epochs): 99.94% usable, 90.1% within 1e-3, **all** disagreements conservative, mean 28 ms vs 812 ms (~29x); baseline + VA 1.1 accuracy preserved |
| VA 1.1 no-regression (accuracy + fix rate) | done (2026-07-18) — 0.028991 m / 0.4727°, 869/1775 fixed vs baseline 876/1775 |
| Implementation (wired into the real AR decision) | experimental only — baseline config remains `rtk_imu_camera_rrr`; VA config is isolated in `research/config/rtk_imu_camera_rrr_va_urbannav.yaml` and must not write to baseline result directories |
| First end-to-end evaluation (first version) | done (2026-07-17) — superseded; the apparent gain was partly a double-count artifact (see review) |
| Independent adversarial code review | done (2026-07-17) — found real double-counting bug + confirmed vision entered only via IMU chain; both since addressed |
| Double-counting fix (single-use local information) | done (2026-07-17) — Medium gain survives (3.21m, beats paper); translation improved on all GICI-board; 3.1 rotation regression shrank (1.300→1.249°) but marginal residual remained |
| **New estimator `rtk_imu_camera_rrr_va` (genuine vision)** | done (2026-07-17) — new registered type, frozen baseline untouched; cPose chain routes reprojection into AR covariance; marginalization-prior crash found+fixed, **ASan-clean (1137 epochs, 0 heap errors)**; vision confirmed entering (num_camera_keyframes>0) |
| Genuine-vision evaluation (single run each) | superseded — promising single-run Medium/GICI-board numbers are not sufficient for a positive claim |
| Repeat-run variance characterization | done for Deep n=3 baseline vs VA — negative on horizontal accuracy; Medium repeat evidence is not available in this workspace because Medium data is absent |
| Epoch-level false-fix validation | done as high-frequency jump check, but insufficient — it missed slow drift-inducing wrong/overconfident fixes |
| Technical design refinement (marginalize neighbors vs condition-fixed; adaptive ratio test) | exact nuisance marginalization implemented for VA covariance; adaptive ratio test still not started; repeat-run validation required before any positive claim |
| Deeper literature pass before submission | not started — 2026-07-17 pass was a first scoping check |
| Decision layer "VA-v2" (joint vision/IMU veto + P_s ≥ 0.999 bootstrap gate) | done (2026-07-18) — implemented, frozen baseline preserved with gates OFF; veto fired 0 times everywhere (harm materializes after acceptance, not at it); P_s gate halved fix rate keeping u/yaw gains |
| **Paired VA-v2 Deep n=3 (clean protocol) + final verdict** | done (2026-07-18) — vertical better 6/6 paired waves (−38..−50%), yaw 5/6 (−40..−60%), fix rate ×2.4–5; horizontal worse 5/6 (median +0.6 m, tail +3.65 m) — across-the-board SOTA claim NOT supported; Medium deltas shown to be protocol noise (0 fixes both sides); next principled step = soft/revocable fix application (new phase, pre-registered) |
| Replay-pacing finding | post-file mode is backpressure-paced (no wall-clock pacing in `FilesReading::run()`); `max_solver_time` is wall-clock ⇒ results are load-sensitive ⇒ within-pair simultaneous execution is mandatory for fair comparison |
