# Vision-Aided Ambiguity Resolution — research branch

**Branch:** `research/vision-aided-ambiguity-resolution`
**Worktree:** `/home/theph/ws_ncs/gici_vision_aided_ar`
**Base:** `research/standard-env` (upstream-clean `f2b8579` + research wrappers only —
deliberately *not* branched from the ROS2 real-time work, since this is a file-mode
algorithmic study that must stay comparable to the paper's own evaluation methodology).
**Goal:** a genuine algorithmic improvement to GICI-LIB's RTK ambiguity resolution (AR),
targeting a real conference/journal submission (not a thesis-only writeup).

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
| Fuse local information with a prior (shadow-estimator covariance) | not started — **current step**; needed because ~45% of epochs are near-singular using local-only information |
| Technical design (how fused cross-info improves AR: decorrelation vs. ratio-test vs. validation) | blocked on above |
| Implementation (wired into the real AR decision, not just diagnostics) | not started |
| Evaluation on locked baselines | not started |
