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
didn't take" — e.g. querying `graph_->computeCovariance()` for the ambiguity IDs
*plus only the current epoch's pose/velocity block* (not the full window), which
should be far cheaper than what the authors deemed infeasible while still capturing
the dominant cross-correlation (most of it should come from the current-epoch
coupling, not distant historical states). Natural minimal-change entry point:
`rtk_imu_camera_rrr_estimator.cpp:387`, replacing the call into the shadow estimator.

Also noted: `AmbiguityResolution::addStableFixationToGraph()` is declared
(`include/gici/gnss/ambiguity_resolution.h:260`) but has no implementation and no call
site anywhere — dead code, apparently intended for constraining stable-fixed
ambiguities across epochs (not cross-sensor coupling as far as the header comment
suggests, but worth re-checking once the design is more concrete).

## Plan

1. **Measure the cost first.** Before designing around "the full joint covariance is
   too expensive," verify that claim quantitatively on this repo's own hardware/window
   sizes: instrument `rtk_imu_camera_rrr_estimator.cpp:387` to call
   `graph_->computeCovariance()` with the ambiguity IDs plus the *current* pose/
   velocity/attitude parameter-block IDs (not the whole sliding window) and time it
   against the existing shadow-estimator path, on UrbanNav Medium (100% float — the
   most informative case) and GICI-board 3.1/4.1. This determines whether the
   "current-epoch-only" cross block is actually cheap enough for real-time use, which
   is the load-bearing assumption of the whole contribution.
2. If (1) is tractable: design how the cross-covariance actually improves AR —
   candidates: (a) reshape the MLAMBDA decorrelation transform with the joint
   information instead of the GNSS-marginal one, (b) tighten/adapt the ratio-test
   threshold based on joint uncertainty, (c) a visual-consistency check on candidate
   fixed solutions computed from the *same* joint covariance (not a separate VIO
   thread, unlike Ran et al. 2026 above). Pick based on which gives the cleanest,
   most defensible story — likely (a) or (b), since (c) starts to resemble the
   existing external-signal approaches in the literature.
3. If (1) is *not* tractable even for the current-epoch-only block: that itself is a
   valid, honest finding (confirms/quantifies the original authors' tradeoff) — pivot
   to a cheaper approximation (e.g. a fixed-lag or diagonal-only cross term) rather
   than abandoning the direction.
4. Implement as an opt-in estimator option (do not change existing locked baselines'
   default behavior) so `verify_upstream_fidelity.sh`-style comparisons stay valid.
5. Evaluate on the existing locked baselines (GICI-board 1.1/3.1/4.1, UrbanNav
   Medium/Deep) — Medium (100% float today) is the primary test case: does fixed-rate
   or accuracy improve without regressing the others?
6. Ablation experiments folded into the same paper (not separate submissions, per
   2026-07-17 discussion): base-RINEX-interval robustness (from the Medium fix) and
   resource-aware real-time behavior (from `research/ros2-live-nosparsify-experiment`)
   as supporting robustness results for the same core contribution, not separate claims.
7. Deeper literature pass before submission (the 2026-07-17 pass above was a first
   scoping check, not exhaustive) — particularly around the GS-GVINS / PO-GVINS /
   TITS 2024 factor-graph-AR line of work, to confirm none of them already do the
   current-epoch cross-covariance approach.

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
| Cost measurement of current-epoch-only cross-covariance | not started — **next step** |
| Technical design | blocked on cost measurement above |
| Implementation | not started |
| Evaluation | not started |
