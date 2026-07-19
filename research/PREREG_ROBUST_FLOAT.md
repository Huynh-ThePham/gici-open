# Pre-registration: bounded-influence GNSS float estimation ("RF", Tukey kernel)

Registered: 2026-07-18 ~22:50, BEFORE implementation is built or any run starts.
Frozen once the first evaluation run starts; deviations via dated addenda.

## What the closed decision-layer ladder established (motivation)

Three pre-registered falsifications (PREREG_SOFT_FIX.md, PREREG_SOFT_WEIGHT.md)
closed the decision layer: gates cannot see the harm (verdict veto fired 0
times ever), expiry destroys the benefits with the harm (same channel), and
decision-confidence weighting (VA-v4) preserves the vertical/yaw/availability
benefits but leaves the horizontal tail (n=3 solo: h = 3.06/3.65/5.23 vs BL
anchor ~3.2/2.5/[run3 pending]). Forensics of the tail runs shows fixes that
are locally sound being anchored to NLOS-BIASED float states (fixes accepted
at P_s ~ 1 while the solution is already meters off; blow-ups in float-only
stretches minutes later via the prior). The remaining root cause is the FLOAT
estimate itself: deep-urban NLOS biases the float solution, every Gaussian
confidence measure saturates blind to it, and constraints anchored to it
persist.

Code fact (verified): the upstream float estimator already applies
`ceres::HuberLoss(1.0)` to all GNSS residuals (pseudorange, phaserange,
doppler) plus hard-threshold outlier rejection. Huber is UNBOUNDED-influence:
a 3-5 sigma NLOS-biased residual retains linear pull on the state. The
untested lever is bounded influence.

## Hypothesis (falsifiable)

**H3.** Replacing the GNSS loss with a redescending, bounded-influence kernel
— Tukey biweight, scale c = 4.685 on the sigma-normalized residuals (the
literature-standard 95%-asymptotic-efficiency constant; not fitted to any
dataset) — suppresses the NLOS-biased minority of measurements enough that
(a) the float solution is materially less biased in the hard stretches, and
therefore (b) the horizontal tail of consistent-covariance vision-aided AR
(VA-v4) disappears while its vertical/yaw/availability benefits remain.

A-priori caveat recorded now: if the deep-urban bias is carried by the
MAJORITY of visible satellites (canyon geometry biasing most lines-of-sight
the same way), no per-residual M-estimator can fix it; H3 failure with a
passing mechanism check would establish that, pointing definitively to
external information (vision NLOS classification, 3DMA) as the only remaining
route. Honest prior: ~15-25% success.

## Intervention (single variable)

New option `use_bounded_influence_gnss_loss` (default false = upstream
byte-identical Huber path). When true, the loss object passed to pseudorange,
phaserange, and doppler residual blocks becomes `ceres::TukeyLoss(4.685)`
instead of `ceres::HuberLoss(1.0)`. Nothing else changes: hard outlier
rejection, error parameters, AR pipeline, dose weighting — all untouched.
The marginal covariance remains consistent automatically (its assembly is
loss-corrected via the Triggs term for whatever loss is attached).

Arms:
- **RF-VA = VA-v4 config + this flag** (the confirmatory arm: does the tail go?)
- **RF-BL = baseline config + this flag** (attribution arm: does bounded
  influence alone move the float baseline?) — run after RF-VA, overnight.
Controls: tonight's BL solo n=3 and VA-v4 solo n=3 (same machine, same solo
protocol, same day).

## Mechanism check (GT-free, mandatory)

Per-epoch log `[rfloss] n_gnss=<N> n_downweighted=<k1> n_zeroed=<k2>` counting
sigma-normalized GNSS residuals with |r| > c/2 (materially downweighted) and
|r| > c (zero influence) at solve end. On Deep the log must show a
non-trivial zeroed fraction in the hard stretches; if k2 ~ 0 everywhere, the
kernel is inert (Huber-region data only) and results must not be interpreted
as testing H3.

## Confirmatory protocol (frozen)

1. Build; frozen-baseline 1.1 guard with flag OFF (locked band).
2. 1.1 smoke with flag ON: accuracy within the frozen band (open-sky data is
   Gaussian; Tukey at 95% efficiency must not degrade it), fix count reported.
3. RF-VA solo n=3 on Deep (machine idle, back-to-back), runner's metric,
   n_matched >= 14000, all runs reported.
4. RF-BL solo n=3 (attribution, overnight).

## Pre-specified decision rules

Baseline reference: tonight's BL solo n=3 mean (B_h, B_u, B_yaw).
- Tail event: any RF-VA run with rmse_h > B_h + 1.0.
- **SUCCESS:** zero tail events AND RF-VA mean rmse_h <= VA-v4 solo mean
  (3.98 m) - 0.5 m AND mean d_h vs B_h <= +0.2 AND u better than B_u in >= 2/3
  runs AND yaw better in >= 2/3.
- **PARTIAL:** zero tail events with u/yaw retained (h between bins).
- **FAILURE of H3:** any tail event with mechanism check passing — and per the
  a-priori caveat this specifically implicates majority-bias geometry,
  closing the per-residual-robustness route.
- **INERT:** mechanism check fails (k2 ~ 0) — fix implementation or report the
  kernel never engages; no scientific conclusion.

After this rung, in-estimator levers are exhausted for this paper: the paper
reports the covariance contribution + the complete falsification record; the
NLOS-information route (vision/3DMA) is future work / the next paper.

## Integrity constraints (binding, unchanged)

GT only in post-hoc evaluation; decision rules frozen before runs; all runs
reported; frozen baseline byte-identical with flag unset.

— ADDENDA —

**2026-07-19 06:35 (RF-VA run1 outcome — user's "one run first" gate).** Frozen
guard PASSED (GICI-board 1.1 = 0.028942 m, within band) → RF build's baseline
path is byte-clean. RF-VA run1 on Deep **crashed at t ≈ 160 s** (1692/60476
solution lines): the estimator diverged and visual init threw
`cv::findFundamentalMat` on degenerate points. Mechanism check is unambiguous
and fires from the FIRST epoch: `[rfloss]` zeroed the MAJORITY or ALL GNSS
residuals at 40/48 logged epochs (25 epochs zeroed 5/5, first event zeroed 3/3
at t=0). Diagnosis: Tukey at c = 4.685 rejects residuals > 4.685 σ, but in deep
urban the natural residual spread at the (imperfect) linearization point already
exceeds that — a several-metre initial-position error at ~1 m pseudorange σ is
already > 4.685 σ — so the redescending kernel zeroes the GNSS block from the
start, starves the float of its position anchor, and diverges. This is the
classic no-basin-of-attraction failure of redescending M-estimators, here
endemic (not merely cold-start) because deep-urban linearization never settles
below the rejection radius.

**Conclusion on H3 as pre-registered: NOT RUNNABLE in this form.** The result is
scientifically informative and consistent with the a-priori caveat: too large a
fraction of GNSS residuals are simultaneously large for any per-residual
weighting to isolate the bad ones — you cannot robustly reject the majority.
This is strong evidence for the paper's thesis that the deep-urban horizontal
error is a majority-affecting phenomenon requiring EXTERNAL scene information
(vision NLOS classification, 3DMA) to identify which measurements to trust.

**Decision point (per user "1 run first, then decide"):** options recorded, user
to choose — (A) close the RF/in-estimator lever here with this finding [the
decision-layer ladder was already closed; this closes the float-estimation lever
too]; (B) one exploratory run with a SOFT bounded-influence loss (Cauchy at its
own 95%-efficiency constant c = 2.3849, which retains a gradient everywhere and
so keeps a basin of attraction) to preempt the reviewer question "did a gentler
robust loss help?" — expected to avoid the crash but not to remove the tail. No
tuning of the Tukey constant (raising c to fit the data would violate the
integrity constraints).

**2026-07-19 07:05 (RF-CVA run1 result + promotion to confirmatory, user-directed).**
The soft variant RF-CVA (= VA-v4 + Cauchy c=2.3849 float loss) run1 completed
crash-free (as predicted — basin preserved; `[rfloss]` down 2–3 / ~70, zeroed
0–2) and, unexpectedly, was the single best Deep run of the entire campaign:
raw ENU h = 2.730 m, u = 1.001 m, yaw = 0.874°, fix 9.7%; ATE SE(3) = 1.972 m,
APE Sim(3) = 1.886 m / 1.009° (vs baseline solo ATE 2.49/2.23, VA-v4 solo ATE
2.36/2.25/3.57). This is the first variant with NO horizontal tail in its trial
AND retained vertical gain.

HONEST INTEGRITY NOTE: run1 was pre-registered as *exploratory* and its result
is now SEEN. The decision to pursue RF-CVA to n=3 is therefore data-driven on
run1 (partial unblinding). To keep the confirmation defensible, runs 2 and 3
are the blind confirmatory set: their bins are FROZEN here, before those runs
execute, and run1 is reported as the (already-seen) first of the three with
this caveat stated in the paper.

RF-CVA confirmatory bins (frozen 07:05, applied to runs 1–3 with run1 disclosed):
- Baseline reference = BL solo mean (n=2 available: 2.869 m; report against it).
- Tail event := any RF-CVA run rmse_h > BL_mean_h + 1.0 (= 3.87 m).
- **SUCCESS (H3'):** zero tail events across runs 1–3 AND RF-CVA mean rmse_h <=
  BL_mean_h + 0.2 (horizontal parity-or-better) AND u better than BL in >= 2/3
  AND yaw better-or-equal in >= 2/3. This would be the first positive horizontal
  result and would reframe the paper from limit-study to positive.
- **PARTIAL:** zero tail events, u/yaw retained, mean h in (BL+0.2, BL+0.6].
- **FAILURE:** any tail event — Cauchy does not remove the majority-bias tail;
  confirms the limit-study conclusion.
Mechanism check remains: `[rfloss]` must show non-trivial down-weighting on Deep
(already confirmed on run1). Attribution arm RF-BL-Cauchy (baseline + Cauchy, no
VA) to be run only if the confirmatory set is SUCCESS/PARTIAL, to separate the
Cauchy-float effect from the VA-v4 dose.

**2026-07-19 07:35 (RF-CVA n=3 confirmatory VERDICT: FAILURE of H3').** Runs
(all crash-free, n_matched 15119): h = 2.730 / **4.254** / 3.555 m
(mean 3.513 ± 0.763), u = 1.001 / 1.579 / 2.115, yaw = 0.874 / 0.670 / 0.728,
fix 9.7 / 11.1 / 7.1 %. **run2 h = 4.254 > BL_mean + 1.0 = 3.869 → TAIL EVENT →
FAILURE.** The promising run1 (h = 2.730) was a lucky draw, as its 0.14 m edge
and n=1 warned; the tail returns at n=3 exactly as the a-priori caveat
predicted. Vertical/yaw are retained (VA-v4 underneath; u better than baseline
in 2/3, yaw ~0.7°), but the Cauchy float robustification adds nothing on the
horizontal — it down-weights only 2–3 of ~70 residuals per epoch, far too gentle
to counter a bias carried by the majority. Attribution arm NOT run (gated on
SUCCESS/PARTIAL, not reached). **The in-estimator campaign is closed:** gates,
expiry, dose, redescending float (crash), and soft float (tail) all fail to
remove the deep-urban horizontal tail. This vindicates the pre-registration
discipline — a single-run claim from run1 would have been wrong — and
definitively establishes the limit: the tail requires external NLOS scene
information (Paper 2). This is the final result; the paper is method + boundary.
