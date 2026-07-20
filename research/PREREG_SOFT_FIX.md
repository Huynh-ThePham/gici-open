# Pre-registration: revocable ambiguity fixes ("VA-v3", soft-fix phase)

Registered: 2026-07-18, BEFORE any implementation run of this mechanism.
Frozen once the first evaluation run starts. Any deviation requires a new dated
addendum below the line "— ADDENDA —"; results obtained under a deviated protocol
must be reported as exploratory, never merged into the confirmatory tables.

## Established facts this builds on (research/VISION_AIDED_AR.md, 2026-07-18)

1. Consistent real-time marginal AR covariance is validated (equivalence to
   `ceres::Covariance`, ~29x, no material overconfidence — 171/177 conservative,
   6 numerical ties ≤7.3e-4).
2. Final paired UrbanNav verdict: vertical better 6/6 paired waves, yaw 5/6,
   fix rate x2.4-5 — but horizontal worse 5/6 (median +0.6 m, worst +3.65 m).
3. Diagnosis: accepted fixes are locally GOOD (median dh at fixed epochs <= 0 vs
   paired baseline); the harm is a heavy tail at FLOAT epochs materializing
   4-20 minutes after the last fix. Acceptance-time gates cannot see it (the joint
   vision/IMU veto fired 0 times in every run).
4. Code-level channel (confirmed): each accepted fix adds an `AmbiguityError`
   residual with information 1e6 (std 0.001 cycles, effectively rigid); when the
   ambiguity state slides out of the window, `addAmbiguityMarginBlocksWithResiduals`
   (and the `kAmbiguityError` entry in `addGnssResidualMarginBlocks`'s type set)
   folds that rigid constraint into the everlasting `MarginalizationError` prior,
   linearized at the then-current estimate. A subtly-biased fix therefore becomes a
   near-rigid permanent anchor: the exact irreversibility that produces the late
   float-epoch tail.

## Hypothesis (mechanistic, falsifiable)

**H1.** The horizontal tail regression is caused by ambiguity-fix DECISION
constraints entering the marginalization prior. Excluding them from the prior —
fixes remain fully hard inside the sliding window but expire (are erased, not
marginalized) when their ambiguity states slide out — removes the tail while
preserving the in-window benefits (vertical, yaw, fix rate).

Rationale for why benefits should survive: the u/yaw gains are produced by fixes
constraining the LIVE window (and, in open sky, fixes are re-established every
epoch, so expiry is immediately replaced by a fresh fix). The prior's MEAN still
carries the fix-corrected trajectory through the marginalized measurement
information; only the rigid 1e6 DECISION weight is withheld, so a wrong fix can
later be pulled back by measurements instead of anchoring the prior forever.

## Intervention (exact, single variable)

New opt-in flag `margin_ambiguity_fix_constraints: false` (name in code:
`GnssEstimatorBaseOptions::margin_ambiguity_fix_constraints`, upstream-default
`true` = current behavior, byte-identical when unset). When `false`:

- In `GnssEstimatorBase::addAmbiguityMarginBlocksWithResiduals`: before collecting
  residuals of the out-sliding ambiguity parameter blocks, REMOVE (erase from the
  graph) every attached residual of type `kAmbiguityError`, and log
  `[softfix] erased N fix constraints at marginalization`. Everything else
  (phaserange, `kRelativeAmbiguityError` continuity, etc.) is marginalized as today
  — those encode measurements/models, not decisions.
- No other change. The AR decision layer stays exactly VA-v2
  (`use_joint_cost_validation: true`, `min_bootstrap_success_rate: 0.999`), the
  covariance path stays `ar_use_fast_marginal_covariance: true`.

Config: `research/config/rtk_imu_camera_rrr_va_urbannav.yaml` clone named
`rtk_imu_camera_rrr_va3_urbannav.yaml`, whose ONLY diff is the new flag. The frozen
baseline config and estimator defaults are untouched.

Safety precondition (checked in code review before running): the current + last AR
epochs' `lane_pair.residual_id` handles must never reference constraints attached
to the marginalized (oldest) ambiguity state; holds whenever the window spans > 2
GNSS epochs (true for all configs used here). Guarded by checking
`Graph::residualBlockExists`-equivalent before erase.

## Mechanism check (GT-free, mandatory)

The `[softfix]` log count must be > 0 on Deep runs (the intervention demonstrably
engages, unlike the veto). If it never engages while fixes were accepted, the
implementation is wrong — fix before evaluating, do not interpret results.

## Confirmatory protocol (frozen)

1. Build Release; frozen-baseline guard on GICI-board 1.1 with the plain
   `rtk_imu_camera_rrr` (must stay within locked tolerance 0.029198 m / 0.470557°
   ± 0.005 / 0.1).
2. GICI-board 1.1 with VA-v3 ON: prediction P2 — accuracy within the same frozen
   tolerance band and fix count within ±5% of VA-v2's (869/1775); open-sky
   performance must not degrade (fixes are refreshed every epoch there).
3. UrbanNav Deep, paired protocol (the only valid one): 3 sequential waves, each
   wave = VA-v3 and a FRESH baseline launched simultaneously, one pair at a time,
   nothing else heavy on the machine. Evaluate post-hoc with the runner's own
   metric (raw ENU vs GT interpolated to solution timestamps, no alignment);
   admit only runs with n_matched >= 14000; report ALL waves, no exclusions.
4. UrbanNav Medium, n=1 sanity only (both sides fix 0% there, so the algorithms are
   logically identical; Medium cannot confirm or refute H1 — established fact #2).

## Pre-specified metrics and decision rules

Noise floor (calibrated from Medium, established): |d| <= 0.2 m horizontal is noise.
Tail-regression event: any wave with d_h(VA-v3 - BL) > +1.0 m.

- **SUCCESS (H1 supported):** zero tail-regression events across the 3 waves AND
  median per-wave d_h in [-inf, +0.2 m] AND vertical d_u < 0 in >= 2/3 waves AND
  yaw d_yaw < 0 in >= 2/3 waves.
- **PARTIAL:** zero tail events but median d_h in (+0.2, +0.6] m (tail cured,
  small systematic horizontal cost remains) with u/yaw retained as above.
- **FAILURE of H1:** any tail-regression event with the mechanism check passing
  (constraints demonstrably erased). This falsifies the prior-channel hypothesis;
  the correct next step is deeper diagnosis, NOT parameter iteration.
- **IMPLEMENTATION FAILURE:** u/yaw gains lost (d_u >= 0 in >= 2/3 waves) —
  suggests fixes were effectively neutered in-window; re-inspect implementation
  before drawing scientific conclusions.

Pre-registered secondary rung (run ONLY if the primary is SUCCESS or PARTIAL and
we want the residual horizontal cost characterized): soft-weighted constraint
variant — replace information 1e6 with the decision-confidence information
1 / var, var = (1 - P_s) * (1 cycle)^2 + P_s * (0.001 cycle)^2 (Gaussian moment
match of the bootstrap decision mixture; derived, not tuned; P_s computed per
subset at acceptance). No other rungs. No threshold sweeps.

## Integrity constraints (unchanged, binding)

No GT anywhere in the estimator or in any decision; GT used only in post-hoc
evaluation with the runner's own metric. No tuning against outcomes: the decision
rules above are frozen before the first run. All runs reported, including failures.
Frozen baselines must remain byte-identical in effect with the flag unset.

— ADDENDA —

**2026-07-18 17:05 (before stage-2 launch).** Stage-1 results and one deviation:
- Frozen baseline guard PASS (0.029008 m / 0.485°, within locked band).
- VA-v3 1.1: APE 0.032251 m / 0.479127° — BOTH within the frozen tolerance band
  (P2 accuracy clause PASS). Mechanism check PASS (2682 `[softfix]` lines; up to
  101 constraints erased per marginalization event).
- DEVIATION on the P2 fix-count clause: fixed epochs 1177/1775 (66.3%) vs the
  written reference 869/1775 ± 5%. Two honest notes: (a) the 869 reference was an
  authoring error — it is VA-v1's full-run count; VA-v2 never had a full 1.1 run
  (only a 934-epoch smoke at 59.9%); (b) even against 59.9%, VA-v3's 66.3% is a
  real increase, direction OPPOSITE to the naive expectation (weaker prior →
  larger float covariance → fewer fixes). Interpretation deferred; recorded as a
  surprising secondary observation, not adjudicated by any frozen rule.
- Decision: PROCEED to stage 2. Rationale: P2's purpose was to catch open-sky
  degradation; accuracy holds within the frozen band (translation +3.2 mm vs
  today's paired guard run, rotation better). The fix-count anomaly does not
  touch any Deep decision rule, which remains exactly as frozen above.

**2026-07-18 18:15 (stage-2 outcome).** Verdict per frozen rules: **IMPLEMENTATION
FAILURE** — no scientific conclusion about H1 is drawn from this run set.
- run2 VA-v3 CRASHED (glog FATAL, `marginalization_error.cpp:606`: "trying to
  marginalize out unconnected parameter block") immediately after a `[softfix]`
  erase event: erasing the fix constraints can leave an out-sliding ambiguity
  block with ZERO remaining residuals, and the marginalizer's connectivity
  invariant is fatal on such blocks. Confirmed implementation bug in the
  intervention as coded, not a property of the hypothesis.
- run1 VA-v3 completed but showed EPISODIC vertical corruption (mean |u| per
  200 s bin: 0.6 → 17.1 → 0.5 → 13.2 m, recovery between episodes), fix rate
  collapsed to 0.3%; run3 was clean and slightly better than its paired baseline
  (d_h −0.24, d_u −0.18). Episodic corruption + crash in a sibling run points to
  the same defect class (ill-formed marginalization around ex-fixed blocks).
- Fix applied before any re-run: blocks left residual-less by the erase are now
  REMOVED from the graph (exact — they carry no information) instead of being
  handed to the marginalizer; instrumentation added (`removed N empty ambiguity
  blocks`).
- Plan: ONE exploratory paired Deep wave (labeled exploratory, never merged into
  confirmatory tables) to sanity-check the repaired implementation. If clean, the
  full 3-wave confirmatory protocol re-runs from scratch; if corruption persists,
  the revocable-fix design is declared unworkable as implemented and this line
  stops (reported honestly).

**2026-07-18 19:50 (final verdict — line closed).** Exploratory attempt 2
(repaired build) was crash-free with the mechanism fully active (21,308
constraints erased, 0 empty-block removals needed), but showed the vertical gain
INVERTED (VA 3.53 vs BL 1.82 m) and fix rate below baseline (1.3% vs 1.9%). The
confirmatory re-run's wave 1 (also crash-free) produced d_h = +1.24 m — a
tail-regression event under the frozen rule — with fix rate 0.1%. Since ANY tail
event with a passing mechanism check decides **FAILURE of H1**, and both
remaining waves cannot change that, the user directed aborting waves 2–3 to save
machine time (verdict unaffected; recorded 19:47). **H1 is falsified in a
stronger sense than anticipated: expiring fix constraints does not just fail to
cure the horizontal tail — it destroys the vertical/yaw benefits, demonstrating
that benefit and harm share the same channel (fix persistence in the prior).**
Successor hypothesis (dose instead of switch) is registered separately in
research/PREREG_SOFT_WEIGHT.md before any of its runs.

**2026-07-18 18:45 (exploratory attempt 1).** The first repaired build crashed a
second way (`marginalization_error.cpp:511`): the empty-block guard removed a
parameter block that the PERSISTENT marginalization prior still references — the
prior residual is temporarily detached from the graph during the margin pass
(`eraseOldMarginalization`), so `graph_->residuals(id)` cannot see it and a
prior-connected block looks residual-less. Correct discriminator added: a block is
removed only if it has no graph residuals AND
`MarginalizationError::isParameterBlockConnected(id)` is false; a residual-less
but prior-connected block is marginalized through the marginalizer (pure prior
information), which is exact. Exploratory wave relaunched with this build.
