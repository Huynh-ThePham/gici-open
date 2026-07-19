# Pre-registration: decision-confidence-weighted fix constraints ("VA-v4", soft weight)

Registered: 2026-07-18 ~19:40, BEFORE implementation is built or any run starts.
Frozen once the first evaluation run starts. Deviations require dated addenda below
"— ADDENDA —"; deviated results are exploratory only.

## What the two prior falsifications established (this is the motivation)

1. **VA-v2 (gates):** filtering WHICH fixes are accepted (joint vision/IMU veto +
   P_s ≥ 0.999 bootstrap gate) does not cure the Deep horizontal tail. The veto
   never fires (harm materializes minutes after acceptance, through the prior);
   halving the fix rate kept vertical/yaw gains but kept the tail too.
2. **VA-v3 (revocable fixes):** making fix constraints EXPIRE with the window
   (erased at marginalization; never enter the prior) removes the catastrophic
   horizontal blow-ups but ALSO destroys the benefits — fix rate collapses below
   baseline and the 6/6-wave vertical gain inverts. Confirmatory wave 1:
   d_h = +1.24 m (tail event), fix rate 0.1%.

Joint conclusion: **the benefit and the harm live in the same channel — the
persistence of fix information in the marginalization prior.** A binary
keep/expire switch cannot separate them. The remaining principled lever is DOSE:
the weight with which a fix persists must reflect the probability that the
decision was correct.

## Hypothesis (falsifiable)

**H2.** Replacing the hard-coded fix-constraint information (std 0.001 cycles,
information 1e6 cycle⁻²) with the decision-confidence information derived from
the integer-bootstrapping success rate P_s of the accepted subset,

    var(P_s) = (1 − P_s) · (1 cycle)² + P_s · (0.001 cycle)²
    information(P_s) = 1 / var(P_s)

preserves the deep-urban vertical/yaw gains (the constraint stays tight:
P_s = 0.999 → std ≈ 0.0323 cycles) while removing the horizontal tail (the prior
is never anchored more strongly than the decision's own error model justifies:
a 0.1%-probable wrong integer contributes bounded, recoverable information).

Derivation note: this is Gaussian moment matching of the two-component decision
mixture (correct w.p. P_s with upstream's baseline measurement variance; wrong
w.p. 1−P_s with error ~1 cycle, the dominant nearest-integer failure mode). It
contains NO free parameters: P_s is computed per accepted subset from the same
float covariance already used by the gate, and both mixture variances are fixed
by the model (upstream's 0.001 cycles; the 1-cycle integer grid).

## Intervention (single variable vs VA-v2)

- VA-v4 = VA-v2 (fast marginal covariance + joint veto + P_s ≥ 0.999 gate,
  `margin_ambiguity_fix_constraints` back at its upstream default TRUE) + ONE
  change: in `AmbiguityResolution::solveLanes` "Add to graph", the constraint
  information becomes `information(P_s)` of the accepted subset instead of the
  constant 1.0e6. New option `use_success_rate_fix_information` (default false =
  upstream byte-identical).
- P_s of the accepted subset: the LAMBDA path stores the gate's
  `bootstrapSuccessRate(active_float_covariance)` value at the accepted subset
  size; the rounding path computes the same function on
  `float_covariance.topLeftCorner(num_active)` (caveat documented: LD-decorrelated
  P_s upper-bounds plain rounding success; accepted as model approximation).
- The within-epoch Kalman update (`fix_ambiguity_covariance = I·1e-6`) is NOT
  touched: it shapes only intra-epoch lane sequencing (the AR-internal covariance
  is rebuilt from the estimator every epoch); the identified harm channel is the
  GRAPH constraint that persists into the prior. Single-variable discipline.
- Mechanism check (mandatory, GT-free): log `[softw] n=<lanes> Ps=<val>
  std_cycles=<val>` at each acceptance; on Deep runs the log must show fixes
  accepted with std materially > 0.001 cycles (i.e., the dose mechanism actually
  engages). If every accepted subset logs std ≈ 0.001, the intervention is inert
  — fix before interpreting.

## Confirmatory protocol (frozen; identical to PREREG_SOFT_FIX.md structure)

1. Build Release; frozen-baseline guard on 1.1 (plain estimator, locked band
   0.029198 m / 0.470557° ± 0.005 / 0.1).
2. VA-v4 on 1.1: accuracy within the same frozen band. Fix count reported (no
   band pre-committed — VA-v3 taught us we cannot predict it; it is reported,
   not adjudicated).
3. UrbanNav Deep, 3 sequential paired waves (VA-v4 + fresh baseline
   simultaneously, one pair at a time, machine otherwise idle), runner's own
   metric, n_matched ≥ 14000, all waves reported.

## Pre-specified decision rules (identical bins to PREREG_SOFT_FIX.md)

Noise floor |d| ≤ 0.2 m (Medium-calibrated). Tail event: d_h > +1.0 m in any wave.

- **SUCCESS:** zero tail events AND median d_h ≤ +0.2 m AND d_u < 0 in ≥ 2/3
  waves AND d_yaw < 0 in ≥ 2/3 waves.
- **PARTIAL:** zero tail events, median d_h in (+0.2, +0.6], u/yaw retained.
- **FAILURE of H2:** any tail event with the mechanism check passing.
- **BENEFIT LOSS:** d_u ≥ 0 in ≥ 2/3 waves — the dose was too weak to matter or
  fixes stopped being accepted; report as H2 unsupported in the tested form.

After this rung the decision-layer ladder is CLOSED regardless of outcome: no
further variants without a fundamentally new mechanism hypothesis. The paper
reports VA-v2 as the main method plus the full falsification chain (gates →
expiry → dose).

## Integrity constraints (binding, unchanged)

No GT anywhere in estimator or tuning; GT only in post-hoc evaluation via the
runner's metric; decision rules frozen before the first run; all runs reported;
frozen baselines byte-identical with flags unset.

— ADDENDA —

**2026-07-18 20:20 (mechanism-check failure on Deep wave 1 → registered remedy,
BEFORE any run of the modified variant).**
- Stage 1 PASSED in full: guard 0.029078 m / 0.4753°; VA-v4 1.1 0.029134 m /
  0.4897° (both in the frozen band); fix count 818/1775 (normal); [softw] active.
- Deep wave 1 (gated variant): crash-free, complete, VA4 h=2.981 u=3.038
  yaw=0.670 fix=3.3% vs BL h=2.445 u=3.038 yaw=0.701 fix=1.6% → d_h=+0.537 (no
  tail event), d_u=0.000, d_yaw=−0.031.
- MECHANISM CHECK FAILED as defined above: every accepted subset logged
  P_s ≥ 0.999995 → std ≤ 0.0024 cycles ≈ upstream-hard. Cause: the inherited
  VA-v2 gate (min_bootstrap_success_rate 0.999) pre-selects subsets until P_s
  saturates, leaving the dose no dynamic range. Stacking gate + dose was an
  authoring error in the intervention spec — the design intent was dose INSTEAD
  of switch.
- Registered remedy (config-only, no rebuild): set
  `min_bootstrap_success_rate: 0.0` in the VA-v4 configs so the decision-
  confidence weight is the SOLE P_s consumer. Everything else unchanged;
  decision rules unchanged. The gated wave-1 pair is kept as an exploratory
  record (it essentially replicates VA-v2 behavior).
- Re-validation required before Deep: VA-v4' on 1.1 must stay in the frozen band
  and the [softw] log must now show materially soft acceptances (std > 0.005
  cycles for some subsets) on Deep.
- Scientific note recorded now (before outcomes): P_s assumes zero-mean Gaussian
  float errors; deep-urban NLOS biases violate this, so 1−P_s underestimates the
  true failure rate. If the ungated dose also fails, that model mismatch is the
  mechanistic explanation to report, and the decision-layer ladder closes.

**2026-07-18 22:00 (final confirmatory set, user-directed; registered before
runs 2-3 of either arm complete).** Protocol evolved once more at the user's
direction for production fidelity and transparency: the confirmatory set is
**n=3 SOLO runs per arm** (machine otherwise idle, back-to-back queue):
VA-v4' (ungated dose) runs 1-3 in `urbannav_va4solo_20260718`, baseline runs
1-3 in `urbannav_blsolo_20260718`. VA-v2/VA-v3 solo (n=1 each) are context rows
only. Scoring: group means ± std + all-pairs deltas; frozen bins applied as:
tail event := any VA-v4 run with rmse_h > (BL mean + 1.0 m); SUCCESS := zero
tail events AND VA-v4 mean d_h ≤ +0.2 m AND u better in ≥2/3 runs vs BL mean
AND yaw better in ≥2/3; PARTIAL/FAILURE analogous to the original bins.
Secondary reporting: p95/max horizontal (deployment profile), and post-hoc
3-tier metrics (raw ENU / ATE SE(3) / APE Sim(3)) for every run. Run-1 results
known at registration time (VA-v4 3.060/0.894/0.642/6.1% vs BL
3.230/1.893/0.802/1.6%); runs 2-3 of both arms are not yet complete and the
bins above are frozen before their outcomes are seen.

**2026-07-18 20:55 (protocol change directed by the user, BEFORE the confirmatory
runs).** The user prefers each run to execute ALONE on an idle machine
(production-faithful conditions) rather than the within-pair simultaneous
protocol. Trade-off acknowledged on record: paired-parallel controls slow
environmental drift but shares the machine (both members mildly solver-starved
vs production); solo-sequential is production-faithful but exposed to drift
(today's measured drift: 14–24 min wall for identical work). Registered
mitigation: INTERLEAVED solo sequence VA₁→BL₁→VA₂→BL₂→VA₃→BL₃, launched
back-to-back automatically, machine otherwise idle; comparisons use adjacent
pairs; decision rules unchanged (tail event d_h > +1.0 m in any adjacent pair;
SUCCESS/PARTIAL bins on the 3 adjacent-pair deltas). The in-flight paired wave 1
(ungated) is kept as exploratory/secondary data only. Mechanism-check status
from it: PASS (26/315 acceptances with std > 0.005 cycles, max 0.615).
