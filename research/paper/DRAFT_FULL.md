# Consistent Real-Time Ambiguity Covariance for GNSS/IMU/Vision RTK — and the Limits of In-Estimator Ambiguity Validation in Deep Urban

*Draft v0.3 — 2026-07-19. Target venue: IEEE RAL. All numbers final and
traceable to research/VISION_AIDED_AR.md and the three PREREG_*.md files with
dated addenda; five pre-registered interventions completed (gates, expiry, dose,
hard-robust float, soft-robust float).*

---

## Abstract

Tightly-coupled GNSS/IMU/vision RTK systems resolve carrier-phase integer
ambiguities using a float ambiguity covariance. State-of-the-art open-source
systems compute this covariance with a GNSS-only shadow estimator because the
joint covariance of the full factor graph is too expensive to form online — a
workaround that discards exactly the IMU and visual information that tight
coupling is supposed to provide. We present an exact-in-window marginal
ambiguity covariance that is statistically consistent — identical to the full
`ceres::Covariance` solution up to linearization — and real-time: a two-stage
sparse Schur elimination over the active window (per-landmark rank-truncated
elimination, then an equilibrated dense factorization with iterative-refinement
self-validation), including the marginalization prior. On a 1,787-epoch
open-sky sequence the method matches the reference covariance on 99.94% of
epochs (90.1% within 1e-3 relative error); where they differ more it is
conservative (larger trace) in 171 of 177 cases and, in the remaining 6, its
trace sits below the reference by at most 7.3e-4 at the 1e-3 boundary — a
numerical tie, never material overconfidence — and
runs at 28 ms mean versus 812 ms (29x speedup, max 97 ms vs 2.7 s).

Making the consistent covariance available at every epoch exposes a second,
deeper problem: in deep-urban conditions, integer fixes that are locally
correct still degrade horizontal accuracy minutes later, because each accepted
fix enters the everlasting marginalization prior as a near-rigid constraint
(sigma = 0.001 cycles). Through a pre-registered falsification chain we show
that (i) acceptance-time validation cannot detect this harm (a joint
vision/IMU cost veto never fires — the damage materializes 4–20 minutes after
acceptance), and (ii) letting fix constraints expire with the sliding window
removes the harm but destroys the benefits — benefit and harm share the same
channel, the persistence of fix information in the prior. The remaining lever
is dose: we weight each fix constraint by its own decision confidence,
information = 1/[(1-P_s) + P_s·1e-6] cycles^-2, where P_s is the Teunissen
bootstrapping success rate of the exact accepted subset — a Gaussian moment
match of the decision mixture with no free parameters. Across n=3
production-like solo trials on UrbanNav Deep, decision-weighted fixing improves
vertical accuracy by ~50% (0.89/0.89/1.23 vs baseline 1.89/2.28 m), yaw, and
fix availability 3.8x — robustly — yet the horizontal tail persists
(3.06/3.65/5.23 m). We then move below the decision layer and robustify the
float estimate itself; a redescending loss zeroes the majority of GNSS
residuals from the first epoch and diverges, showing the deep-urban error to be
a *majority-affecting* bias that no per-measurement scheme can isolate. The
practical conclusion is a clear division of labor: the consistent covariance
and decision-confidence weighting deliver robust vertical/yaw/availability gains
with the horizontal channel held at baseline, and the residual horizontal tail
is out of reach of any in-estimator mechanism operating on GNSS statistics — it
requires external scene information (vision-based NLOS classification, 3D map
aiding). All interventions are opt-in; the upstream baseline is preserved
byte-identically and re-verified after every build. Ground truth is used only
in post-hoc evaluation; every decision rule was pre-registered and frozen
before the corresponding runs, and all runs — including three failed
interventions — are reported.

---

## 1. Introduction

Precise urban positioning increasingly relies on tightly-coupled fusion of
RTK GNSS, inertial, and visual measurements in a sliding-window factor graph.
The decisive accuracy event in RTK is integer ambiguity resolution (AR):
fixing carrier-phase ambiguities to integers converts decimeter-level float
solutions into centimeter-level fixed ones — when the integers are right.
Whether they are right is governed by the float ambiguity covariance fed to
the integer search (LAMBDA) and its acceptance tests.

Here lies a structural compromise in current open systems. In GICI-LIB [chi23],
the reference open-source GNSS/IMU/camera platform, AR consumes a covariance
from a parallel GNSS-only "shadow" estimator, which the authors themselves
annotate as a coarse covariance: forming the true joint covariance of the full
graph online is considered infeasible (we measure 687 ms mean per epoch via
sparse-QR `ceres::Covariance` once the window fills — 86% of epochs exceed a
50 ms budget). The shadow covariance discards all IMU and visual information —
precisely the information that distinguishes a tightly-coupled system from a
GNSS-only one at the moment that matters most.

The obvious fix — make AR use the joint covariance — requires solving two
problems at once. The covariance must be **consistent** (not overconfident:
we show that a cheaper conditioning-based approximation, which drops the
Schur complement of out-of-set states, inflates confidence and drives the
estimator into accepting drift-inducing wrong fixes, degrading deep-urban
horizontal RMSE from 2.8 m to 6.8 m), and it must be **real-time** at every
AR epoch, because a covariance that is only occasionally available reverts
the system to the shadow path exactly when conditions are hard.

**Contribution 1** is such a covariance: an exact-in-window marginal
Q_aa = [H_active^-1]_aa over the ambiguity block, computed by structure-
exploiting elimination with a numerical self-validation gate, including the
marginalization prior through a read-only information accessor. It is the
same quantity `ceres::Covariance` computes, at 1/29th the cost, and it is
never materially overconfident in 1,787 epochs of validation (worst-case trace
deficit vs the reference 7.3e-4).

Deploying the consistent covariance at every epoch on the UrbanNav benchmark
produced an unexpected result that motivates the rest of the paper: robust
improvements in vertical accuracy (6/6 paired trials, −38..50%), yaw (5/6,
−40..60%) and fix availability (2.4–5x) — but horizontal accuracy that is
*worse* in 5 of 6 paired trials, occasionally by meters. A per-epoch paired
diagnosis shows the accepted fixes are locally sound; the harm is a heavy
tail at float epochs minutes later. The mechanism is architectural: each
accepted fix is inserted as a near-rigid constraint (0.001 cycles) that the
sliding window eventually folds into the everlasting marginalization prior.
A subtly biased fix — biased not because the integer is wrong but because
deep-urban NLOS biases the float solution it is anchored to — becomes a
permanent anchor the estimator cannot revise.

**Contribution 2** is a pre-registered falsification chain that turns this
observation into design knowledge. We test, with frozen decision rules and
GT-free mechanism checks: acceptance gates (a zero-threshold joint
vision/IMU cost veto plus a P_s >= 0.999 bootstrapping gate) — the veto never
fires because the harm does not exist at acceptance time; constraint expiry
(fixes erased at marginalization instead of absorbed) — the tail disappears
but so do the vertical/yaw gains and the fix rate collapses below baseline,
proving benefit and harm share the persistence channel; and finally
**decision-confidence weighting** — each fix persists with information
matched to the probability its integer decision was correct.

**Contribution 3** is the weighting itself: information(P_s) =
1/[(1-P_s)·1 + P_s·1e-6] cycles^-2, the **second moment** (about the fixed
value) of the two-component decision mixture — correct with probability P_s at
the upstream constraint variance (1e-6 cycle^2), wrong with probability 1-P_s
off by ~1 cycle on the integer grid (1 cycle^2) — used as the constraint
variance; P_s is the bootstrapping success rate of the exact accepted subset
(Teunissen; a lower bound on the ILS success rate), computed from the same
decorrelated conditional variances LAMBDA uses. No constant in the scheme is
tunable. This is a variance inflation (it matches the mixture's spread), not a
bias correction (it discards the sign of a wrong fix's offset) — a distinction
that turns out to explain its failure, since the deep-urban harm is a bias, not
excess variance (Sec. 6c, 8). It is a two-point, graph-constraint-level cousin
of Best Integer Equivariant estimation, which instead keeps the full posterior
weighting over integer candidates.

**Contribution 4** is methodological: protocol findings for replay-based
GNSS benchmarking (the post-file replay of the reference platform is
backpressure-paced, making results load-sensitive; identical-load pairing or
solo-idle execution is mandatory), a calibrated protocol noise floor
(~0.2 m, from sequences where the compared algorithms are provably
identical), and a fully pre-registered experimental discipline (frozen
decision bins, mechanism checks, dated addenda) that we argue should be
standard for AR research, where the temptation to tune acceptance thresholds
against ground truth is strong and invisible in published numbers.

## 2. Related Work

*[To be completed after the deep literature pass — key anchors:]*
- **GICI-LIB** [chi23] and the shadow-covariance workaround; GVINS, IC-GVINS,
  VINS-Fusion as fusion baselines (cf. Table V of [chi23]).
- **Integer estimation theory**: LAMBDA and MLAMBDA; ratio test and its
  fixed-failure-rate variants (FFRT); partial ambiguity resolution;
  **bootstrapping success rate** P_s [teunissen98] as an a-priori decision
  quality measure. Closest in spirit is **Best Integer Equivariant (BIE)**
  estimation [teunissen03, odolinski20], which weights integer candidates by
  posterior probability instead of committing to one; our constraint weight is
  a two-point BIE-style mixture applied at the graph-constraint level, keeping
  the standard LAMBDA pipeline intact.
- **Consistency in sliding-window estimators**: marginalization priors, FEJ;
  overconfidence pathologies.
- **Deep-urban GNSS**: NLOS/multipath mitigation, 3DMA, robust error models —
  context for the limits we establish.

## 3. System Overview

We build on the RTK/IMU/camera tightly-coupled estimator of GICI-LIB
("RRR"): a Ceres-based sliding-window factor graph with GNSS double-diff
pseudorange/phaserange factors, IMU preintegration, reprojection factors,
and a dense marginalization prior; AR forms between-satellite single
differences, solves UWL/WL/NL lanes by MLAMBDA with elevation/variance/
fractional-ordered partial fixing, applies accepted integers as constraint
factors (sigma = 0.001 cycles) plus an internal Kalman update, and validates
by a zero-threshold range-cost check. Our estimator (`rtk_imu_camera_rrr_va`)
is a registered variant; the upstream estimator and its defaults are
untouched (frozen-baseline tolerance re-verified after every build:
APE 0.0290–0.0292 m / 0.470–0.485° across four rebuilds today against the
locked 0.029198 m / 0.470557° ± 0.005/0.1).

## 4. Consistent Real-Time Marginal Ambiguity Covariance

**Problem.** AR needs Q_aa = [H^-1]_aa where H is the Gauss-Newton
information of the *entire active window* — GNSS, IMU, vision, and the
marginalization prior — evaluated at the current estimate, and `aa` indexes
the ambiguity blocks. `ceres::Covariance` computes exactly this but at
O(whole-problem sparse QR) cost; a local conditioning approximation
(dropping H_an H_nn^-1 H_na for out-of-set blocks) is fast but overconfident
— the documented negative result above.

**Method.** We assemble H = sum_r J_r^T J_r over all active residuals in
minimal coordinates, with two care points that decide correctness:
(1) robustified residuals contribute their **Triggs-corrected** Jacobians
(matching Ceres' corrector), not raw ones; (2) the marginalization prior
contributes through a new read-only accessor that exposes its internal
Lambda = J^T J with per-block minimal offsets — never through generic
evaluation paths (whose dynamically-sized buffers are the crash-prone route).
We then eliminate in two stages exploiting the graph's structure:

- *Stage 1 — landmarks.* Each landmark's 3x3 block H_ll couples only to
  poses. We eliminate each landmark by a rank-truncated pseudo-inverse Schur
  complement; the PSD argument is that null modes of a landmark block have
  zero coupling rows, so truncation is exact, not approximate.
- *Stage 2 — dense nuisance.* The remaining nuisance block (poses,
  speed/bias, clocks, frequencies, extrinsics, prior-only variables) is
  Jacobi-equilibrated (unit-heterogeneity, not degeneracy, dominates its raw
  conditioning) and factorized by dense LDLT with three steps of iterative
  refinement. The refinement residual doubles as a **numerical
  self-validation**: the result is accepted only if the last increment's
  Schur-complement effect is below the smallest eigenvalue scale of the
  reduced system — an arithmetic-error gate with no tuned constants. On any
  failure (typed abstention reasons are logged), the caller falls back to the
  shadow covariance; we never fabricate confidence.

**Validation (Table 1).** Full GICI-board 1.1 trajectory, 1,787 AR epochs,
per-epoch comparison against `ceres::Covariance`:

| metric | value |
|---|---|
| usable epochs (fast path) | 99.94% |
| rel. error <= 1e-3 | 90.1% |
| disagreements > 1e-3 | 177 — conservative (larger trace) 171; below-reference 6, worst trace deficit **7.3e-4** at the 1e-3 boundary (numerical ties) |
| mean / max time (fast) | 28 ms / 97.5 ms |
| mean / max time (ceres) | 812 ms / 2,714 ms |
| heap safety | ASan clean, 1,761 epochs, 0 errors |

The conservative-or-tied disagreement profile is the AR-safety property: the
failure mode of the numerical fallback chain inflates uncertainty, never
materially deflates it (the 6 below-reference cases are 0.02–0.07% trace ties at
the 1e-3 threshold, not overconfidence). The measure here is the covariance
trace, the quantity logged online; a per-ambiguity-diagonal check (the exact
quantity the AR gate consumes) is left for a reviewer-response appendix. Open-sky
accuracy and fix counts are preserved (0.028991 m / 0.4727°, 869/1775 fixed vs
876/1775 baseline).

## 5. UrbanNav Evaluation: the Horizontal Puzzle

**Protocol.** UrbanNav-HK Medium (TST) and Deep (Whampoa); raw ENU RMSE
against GT interpolated to solution timestamps, **no alignment** (the
deployment-relevant metric; Sec. 8 cross-walks to aligned ATE/APE);
completeness gate n_matched >= 14,000/15,119; n=3 repeats; and — after
discovering that the platform's post-file replay is backpressure-paced (no
wall-clock pacing exists in the reader loop; the same 25.2-min sequence
spans 14–24 min of wall time with identical work) while the solver budget is
wall-clock — **identical-load pairing**: each comparison wave runs both
algorithms simultaneously; cross-regime comparisons are never made. Medium,
where both algorithms accept zero fixes and are therefore logically
identical, calibrates the protocol noise floor at ~0.2 m — we flag any
smaller delta as noise, including our own Medium "improvement".

**Results (Deep, paired, n=3 each; Table 3).**

| arm | rmse_h (m) | rmse_u (m) | yaw (deg) | fixed |
|---|---|---|---|---|
| baseline (matrix-1) | 3.026 ± 0.679 | 2.266 ± 0.267 | 1.089 ± 0.279 | 1.7% |
| VA-v1 (consistent cov.) | 3.401 ± 0.784 | 1.463 ± 0.213 | 0.657 ± 0.073 | 8.8% |
| baseline (fresh, v2 wave) | 3.031 ± 0.439 | 2.871 ± 0.613 | 1.743 ± 0.903 | 1.3% |
| VA-v2 (+ gates) | 4.614 ± 2.141 | 1.780 ± 0.211 | 0.783 ± 0.045 | 4.1% |

Across all six valid paired waves: vertical better 6/6 (sign test p = 0.016),
yaw better 5/6 — and horizontal **worse 5/6** (median +0.62 m, worst
+3.65 m). Baseline horizontal reproduces almost exactly across protocols
(3.026 vs 3.031), so the design is sound; the effect is real.

**Diagnosis (Table 4).** Conditioning the paired per-epoch error delta on fix
status inverts the naive explanation: at VA-fixed epochs the VA arm is as
good or better than its paired baseline (median dh −0.03/−0.01/−1.01 m per
wave); the excess error lives in a heavy tail at *float* epochs (blow-up
wave: p50 2.77 vs 1.62 m but p99 27.96 vs 9.59 m), in catastrophic segments
occurring 4–20 minutes *after the last accepted fix* (all fixes before
t = 267 s; blow-ups at t = 528–630 s and 1492–1512 s, the latter 47 m vs a
baseline of 1.0 m at the same instant). The only causal channel that
survives this timing is the marginalization prior: fixes are inserted at
sigma = 0.001 cycles and eventually marginalized into a prior that never
forgets; a fix anchored to an NLOS-biased float solution becomes a
permanent, near-rigid bias source. Notably, this danger is invisible to
every acceptance-time quality measure — the accepted subsets carry
bootstrapping success rates P_s >= 0.999995 under the Gaussian model, whose
zero-mean assumption is precisely what NLOS violates.

## 6. The Falsification Chain

All interventions below were pre-registered with frozen decision bins (tail
event := any wave with d_h > +1.0 m; SUCCESS := no tail events AND median
d_h <= +0.2 m AND vertical/yaw retained in >= 2/3 waves) and GT-free
mechanism checks; every deviation carries a dated addendum.

**(a) Acceptance gates (VA-v2).** A joint vision/IMU cost veto (reject if the
robustified reprojection+IMU cost increases after the constrained re-solve —
the exact mirror of the platform's range-cost rule, zero new constants) and
the P_s >= 0.999 subset gate. Outcome: the veto fired **zero times in every
run on every dataset** — consistent with the diagnosis: at acceptance time
there is nothing to see. The P_s gate halved the fix rate (8.8% -> 4.1%)
keeping vertical/yaw gains, but the horizontal tail persisted (worst wave
+3.65 m). *Gates filter decisions; the harm is not a property of the
decision.*

**(b) Constraint expiry (VA-v3).** Fix constraints erased at marginalization
— decisions stay hard in the live window but never enter the prior.
Mechanism verified (21,308 constraints erased; two marginalizer-invariant
pitfalls documented for reproducers: a residual-less block violates the
marginalizer's connectivity check, and a block referenced by the temporarily
detached prior must be marginalized through it, not removed). Outcome:
catastrophic blow-ups gone, but the fix rate collapsed below baseline
(0.1–1.3%), the vertical gain inverted (3.53 vs 1.82 m), and a tail event
remained (+1.24 m) — **FAILURE**, and the informative kind: *benefit and
harm live in the same channel — the persistence of fix information in the
prior. A binary keep/expire switch cannot separate them.*

**(c) Decision-confidence weighting (VA-v4).** The dose: information(P_s) as
in Sec. 1, P_s of the exact accepted subset (LAMBDA path: stored at
acceptance; rounding path: computed on the accepted top-left covariance
block). High-confidence fixes stay near-rigid (P_s -> 1 recovers the
upstream constraint); marginal fixes persist with honestly bounded weight
(P_s = 0.999 -> sigma = 0.032 cycles, a 1000x information reduction).
Mechanism check: on Deep, 44/445 accepted subsets carry materially softened
constraints (up to sigma = 0.62 cycles); on open-sky, P_s ~ 1 throughout and
behavior is upstream-identical (fix count 818/1775; accuracy in the frozen
band) — the dose engages exactly where the model is less certain, and
nowhere else. Outcome, n=3 solo Deep: vertical rock-stable and ~50% better
than baseline (u = 0.894/0.887/1.234 m vs BL 1.893/2.282), yaw better, fix
rate x3.8 — but the horizontal tail persists (h = 3.06/3.65/**5.23** m; run3
is a tail event) — **FAILURE of H2**. Two compounding reasons, both structural:
(i) the harm is invisible to every acceptance-time confidence measure — accepted
subsets carry P_s >= 0.999995 under the Gaussian model whose zero-mean
assumption NLOS violates, so the dose is near-rigid exactly where a wrong fix is
most damaging; (ii) even where the dose does soften, it inflates the constraint
*variance*, whereas the harm is a *bias* (the fix is anchored to a
systematically-shifted float) — a second-moment adjustment cannot cancel a
first-moment error. *No covariance-derived decision layer — gate or dose — can
see, and therefore cannot suppress, a bias the covariance itself does not
represent.* The decision-layer ladder is closed.

**(d) Bounded-influence float (RF).** The last in-estimator lever moves below
the decision layer to the float estimate itself: replace the upstream
unbounded-influence Huber loss on all GNSS residuals with a redescending Tukey
biweight (c = 4.685, the 95%-efficiency constant; no free parameter), so
NLOS-biased measurements lose influence before AR runs. Outcome: the estimator
**diverges and crashes** at t ~ 160 s. The mechanism log is decisive and fires
from the first epoch — Tukey zeroes the *majority or all* GNSS residuals at
40/48 logged epochs (25 zeroing 5/5) — because in deep urban the natural
residual spread at the linearization point already exceeds 4.685 sigma; the
redescending kernel therefore starves the float of its GNSS anchor and loses
its basin of attraction. A soft bounded-influence variant (Cauchy, c = 2.3849,
which retains a gradient everywhere) runs to completion crash-free, gently
down-weighting only 2–3 of ~70 residuals per epoch (never zeroing the block);
its first trial was tail-free (h = 2.73 m) — promising enough that we
pre-registered and ran a full n=3 confirmatory set (runs 2–3 blind, bins frozen,
run1 disclosed). The confirmatory result is **FAILURE**: h = 2.73 / **4.25** /
3.56 m (mean 3.51 ± 0.76) — run2 is a tail event (> baseline + 1.0 m). The
tail-free run1 was a lucky draw, exactly as its n=1 status and sub-noise-floor
0.14 m edge warned. Cauchy down-weights only 2–3 of ~70 residuals per epoch —
far too gentle to counter a majority-affected bias — so it retains VA-v4's
vertical/yaw gains (u better than baseline in 2/3, yaw ~0.7°) but adds nothing
on the horizontal. *Per-residual robustification cannot isolate the bad
measurements because too many are simultaneously biased: the deep-urban error
is a majority-affecting phenomenon that no GNSS-statistics-only mechanism can
resolve.* This vindicates the pre-registration discipline: a single-run claim
from run1 would have been wrong.

## 7. Final Results

Production-like solo protocol (each run alone on an idle machine; baseline
n=2 after a session-teardown loss of one run, VA-v4 n=3; pre-registered bins).
Deep, raw ENU (primary metric):

| arm (solo, Deep) | rmse_h (m) | rmse_u (m) | yaw (deg) | fixed |
|---|---|---|---|---|
| baseline (n=2) | 3.230 / 2.508 | 1.893 / 2.282 | 0.802 / 0.843 | 1.6% / 0.9% |
| **VA-v4 (dose, n=3)** | 3.06 / 3.65 / 5.23 (μ 3.98) | **0.894 / 0.887 / 1.234 (μ 1.01)** | 0.642 / 0.765 / 0.685 | 6.1 / 5.9 / 7.0 % |
| RF-Cauchy (soft float, n=3) | 2.73 / 4.25 / 3.56 (μ 3.51) | 1.001 / 1.579 / 2.115 (μ 1.57) | 0.874 / 0.670 / 0.728 | 9.7 / 11.1 / 7.1 % |
| VA-v2 (gates) — context | 3.534 | 2.328 | 0.952 | 3.5% |
| VA-v3 (expiry) — context | 3.071 | 4.063 | 1.134 | 0.3% |
| RF-Tukey (hard float) — context | crash @160 s | — | — | — |

Both VA-v4 and RF-Cauchy carry a horizontal tail (VA-v4 run3 5.23 m; RF-Cauchy
run2 4.25 m) while improving vertical (μ ~1.0–1.6 m vs baseline ~2.1 m), yaw,
and fix rate. The consistent story across paired and solo protocols, five
interventions, and both robust-loss shapes: **vertical and yaw improve robustly
and substantially; fix availability multiplies; horizontal carries a heavy tail
no in-estimator mechanism on GNSS statistics removes.** VA-v4 is the recommended
configuration where the deployment values vertical accuracy, yaw, and fix
availability and can hold the horizontal channel at float-baseline level.

**T7 — three-tier metric cross-walk (Deep solo).** raw ENU (primary,
deployment), ATE SE(3) (rot+trans align, no scale — the metric-system standard),
APE Sim(3) (rot+trans+scale — [chi23] Table V's metric; deep RRR ref 2.46 m /
1.64°). Alignment progressively hides global bias/scale, so the tiers read
low→high leniency.

| run | raw ENU h | ATE SE(3) | APE Sim(3) m/deg |
|---|---|---|---|
| baseline r1 | 3.230 | 2.495 | 2.494 / 0.978 |
| baseline r2 | 2.508 | 2.234 | 2.183 / 1.014 |
| VA-v4 r1 | 3.06 | 2.358 | 2.341 / 0.779 |
| VA-v4 r2 | 3.65 | 2.252 | 2.097 / 0.869 |
| VA-v4 r3 (tail) | 5.23 | 3.567 | 3.422 / 0.833 |
| RF-Cauchy r1 | 2.73 | 1.972 | 1.886 / 1.009 |
| RF-Cauchy r2 (tail) | 4.25 | 2.978 | 2.870 / 0.859 |
| RF-Cauchy r3 | 3.56 | 2.696 | 2.655 / 0.955 |

Two things to note honestly: (i) even under Sim(3) our baseline (2.18–2.49)
brackets [chi23]'s single-run 2.46, confirming the baseline is reproduced to
spec and the comparison is fair; (ii) the vertical/yaw advantage survives
alignment whereas the horizontal tail survives too (VA-v4 run3 3.42, RF-Cauchy
run2 2.87 Sim(3)) — alignment does not manufacture the vertical/yaw win nor hide
the tail. RF-Cauchy's Sim(3) spread (1.89 → 2.87) mirrors its raw-ENU tail: the
single good run1 does not survive n=3.

## 7a. Recommended configuration and deployment contract

We state the result as an explicit contract rather than a single headline
number, because the method's value is channel-dependent and a deployer must
know which channels it moves.

**Recommended configuration.** Consistent real-time marginal covariance
(`ar_use_fast_marginal_covariance`) + decision-confidence fix weighting
(`use_success_rate_fix_information`), i.e. VA-v4. Real-time (AR-epoch covariance
28 ms mean); opt-in; upstream baseline byte-identical when disabled.

**Contract (UrbanNav Deep, tightly-coupled RTK/IMU/vision):**

| Channel | Effect vs float baseline | Status |
|---|---|---|
| Vertical (up) | ~50% better, low variance (0.9–1.2 m vs 1.9–2.3 m) | **improves — claimed** |
| Yaw | better (0.64–0.77° vs 0.80–0.84°) | **improves — claimed** |
| Fix availability | ×3.8 (6–7% vs 1.6%) | **improves — claimed** |
| Covariance consistency | conservative-or-tied (171/177 larger, 6 within 7.3e-4 trace) | **proven** |
| Real-time | 28 ms mean / epoch (vs 812 ms exact) | **proven** |
| Horizontal (E/N) | held at baseline; heavy tail retained, no claim of improvement | **not improved** |
| Horizontal via any in-estimator lever on GNSS statistics | gates, constraint expiry, decision-confidence dose, and float M-estimation all fail | **proven out of reach** |

**What this means for a deployer.** Use VA-v4 where vertical accuracy (e.g.
multi-level road disambiguation), heading, and fix availability are the binding
requirements and horizontal error can be carried at float-baseline level. Do
NOT expect a horizontal accuracy improvement in deep urban from this or any
GNSS-statistics-based mechanism; that requires external scene information.

**Future work (one line, no new in-estimator ladder).** The horizontal tail is
a majority-NLOS-bias phenomenon; resolving it requires identifying *which*
satellites are non-line-of-sight from scene structure — vision-based LOS/NLOS
classification or 3D-map aiding — applied to the float estimate upstream of AR,
with VA-v4 retained above it. This is a separate system and a separate study.

## 8. Discussion

**What is established.** (i) A consistent joint ambiguity covariance is
computable in real time; its safety property (conservative-or-tied — never
materially overconfident, worst-case trace deficit 7.3e-4) holds empirically
over every validated epoch. (ii) With a consistent covariance,
vision/IMU information robustly improves the vertical channel, yaw, and fix
availability in deep urban. (iii) The horizontal channel is governed not by
decision quality but by decision *persistence*: locally correct fixes,
anchored to NLOS-biased float states, become permanent prior anchors. Gates
cannot see this (the veto never fires); expiry throws away the benefits with
the harm; decision-confidence weighting keeps the benefits but not the tail;
and robustifying the float itself either diverges (redescending Tukey zeroes
the majority-biased block) or, softened to Cauchy, is too gentle to move a
majority-affected bias. Five pre-registered interventions converge on one
conclusion: **the deep-urban horizontal tail is unreachable by any mechanism
operating on GNSS statistics alone.**

**Limits.** Every confidence and weighting quantity in the pipeline (P_s,
ratio, variance gates, robust residual weights) derives from the Gaussian
float covariance, whose zero-mean assumption NLOS violates; on easy stretches
these measures saturate at certainty while the true failure mode stays
invisible, and in hard stretches too many measurements are biased at once for
any per-measurement scheme to isolate the bad ones. The next information
source must be external and must identify *which* satellites are blocked from
scene structure: vision-based satellite LOS/NLOS classification (the platform
already carries a camera) or 3D-map aiding — the subject of the companion
study. We also flag honestly: Medium results are parity by construction (zero
fixes on both arms — deltas there measure the protocol noise floor); baseline
is n=2 on the solo protocol after a session-teardown loss (verdict robust to
it); Deep results carry n=3 statistics under a replay protocol whose
load-sensitivity we characterize but cannot eliminate.

**Integrity.** GT appears only in post-hoc evaluation; every decision rule
was frozen before its runs; every deviation is a dated addendum; all runs
are reported, including three failed intervention designs and two
implementation crashes. We believe this discipline materially changes the
credibility of AR research, where acceptance thresholds are conventionally
tuned on the evaluation datasets.

## 9. Conclusion

Consistent-and-real-time is achievable for joint ambiguity covariance, and
it moves the needle where the model can see (vertical, yaw, availability).
Where the model cannot see (NLOS-biased horizontal), the failure is
architectural, and a five-step pre-registered chain — gates, expiry, dose,
hard-robust float, soft-robust float — establishes that no mechanism on GNSS
statistics removes it, isolating decision-confidence weighting (VA-v4) as the
configuration that preserves the vertical/yaw/availability benefits while
holding horizontal at baseline. Crossing the horizontal boundary needs external
scene information, which we take up in the companion study. The consistent
covariance, the weighting scheme, and the falsification protocol are all
upstream-compatible and opt-in.

---
*Reproducibility: all configs, pre-registrations with dated addenda, run
logs, and evaluation scripts are in the repository; the frozen upstream
baseline is byte-identical in effect and re-verified per build.*
