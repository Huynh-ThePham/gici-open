# Paper draft outline — target: RAL (primary), ION GNSS+ (fallback/companion)

Working title (SUCCESS variant):
  "Consistent Real-Time Ambiguity Covariance for Vision-Aided RTK:
   Why Decision Confidence Must Weight, Not Gate, Integer Fixes"
Working title (PARTIAL/limit variant):
  "Consistent Real-Time Ambiguity Covariance for GNSS/IMU/Vision RTK,
   and the Limits of Decision-Layer Ambiguity Validation in Deep Urban"

## Contributions (claim-checked against evidence on file)

1. **Exact-in-window marginal ambiguity covariance in real time.**
   Two-stage sparse Schur elimination (per-landmark rank-truncated pinv ->
   equilibrated dense LDLT with iterative-refinement self-validation), including
   the marginalization prior via a read-only information accessor. Validated
   per-epoch against ceres::Covariance on 1787 AR epochs: 99.94% usable, 90.1%
   within 1e-3 rel. err., ALL 177 disagreements conservative (never
   overconfident), mean 28 ms vs 812 ms (~29x), max 97 ms vs 2.7 s.
   [EVIDENCE: research/VISION_AIDED_AR.md "Consistent + real-time marginal
   covariance"; logs gici_board_va/1_1]

2. **Decision-confidence-weighted fix constraints (VA-v4).** Integer fixes enter
   the graph (and hence the marginalization prior) with information
   1/[(1-P_s)+P_s*1e-6] cycle^-2 -- Gaussian moment match of the bootstrapping
   decision mixture; no free parameters. [VERDICT: FAILURE of H2 — n=3 solo
   h=3.06/3.65/5.23, tail persists; u/yaw/fix robustly retained.]

3. **A pre-registered falsification chain establishing WHY dose is the right
   lever** (and, in the PARTIAL variant, the limits of any decision layer):
   - Acceptance gates (joint vision/IMU veto + P_s>=0.999): veto fires 0 times
     ever; harm materializes minutes after acceptance through the prior.
   - Constraint expiry (revocable fixes): removes the tail AND the benefits --
     benefit and harm live in the same channel (fix persistence in the prior).
   - Dose (VA-v4): FAILURE — tail persists; benefits robust.
   - Hard-robust float (Tukey): crash (majority-zeroing). Soft-robust (Cauchy):
     FAILURE — n=3 tail returns (run1 was a lucky draw).
   Diagnosis: accepted fixes are locally good (median dh at fixed epochs <= 0 vs
   paired baseline); the harm is a heavy float-epoch tail 4-20 min later.

4. **Evaluation-protocol findings for replay-based GNSS benchmarking:**
   post-file replay is backpressure-paced (no wall-clock pacing) + wall-clock
   solver budget => load-sensitive results => within-pair simultaneous execution
   or solo-idle protocol required; Medium (0-fix) paired deltas calibrate the
   protocol noise floor (~0.2 m).

## Section plan

1. **Introduction** — RTK AR in deep urban; the shadow-estimator workaround in
   GICI-LIB (author-acknowledged "coarse covariance"); gap: consistent AND
   real-time joint covariance; contributions list.
2. **Related work** — [NEEDS DEEP PASS] BIE (Teunissen; Odolinski) as the
   closest theory to soft integer use; ratio test / FFRT / PAR; GNSS/VIO fusion
   (GICI-LIB, GVINS, IC-GVINS, GS/PO-GVINS); marginalization consistency (OKVIS,
   FEJ); NLOS/3DMA context for the limit discussion.
3. **System overview** — GICI-LIB RRR factor graph, AR pipeline (BSD, lanes,
   LAMBDA, partial AR), where ambiguity covariance enters.
4. **Consistent real-time marginal covariance** — derivation; two-stage
   elimination; prior inclusion; loss-corrected (Triggs) J^T J; numerical
   self-validation (no tuned constants); complexity.
5. **Equivalence & cost study (GICI-board 1.1)** — table: usability, rel.err
   distribution, conservativeness, timing vs ceres; frozen-baseline preservation.
6. **Vision-aided AR with consistent covariance (VA-v1/v2) on UrbanNav** —
   paired protocol; results table (BL/VA-v1/VA-v2, n=3): vertical -38..50% 6/6,
   yaw 5/6, fix rate x2.4-5, horizontal worse 5/6 => the puzzle.
7. **Diagnosis** — fixed-epoch vs float-epoch conditioning; segment analysis;
   the prior channel; why acceptance-time validation cannot work.
8. **The falsification chain** — VA-v2 gates, VA-v3 expiry (incl. the two
   marginalizer-invariant pitfalls as implementation notes), VA-v4 dose;
   pre-registration methodology (frozen bins, mechanism checks, addenda).
9. **Final results** — DONE: n=3 solo table (BL/VA-v4/RF-Cauchy) + deployment
   contract (§7a) + 3-tier crosswalk (raw ENU / ATE SE(3) / APE Sim(3) vs Table
   V; baseline Sim(3) brackets [chi23]'s 2.46). Verdict: limit-study /
   method+partial-win+boundary.
10. **Discussion** — what the chain establishes; integrity/consistency as a
    deployment property (never-overconfident covariance); limits: Gaussian
    confidence saturates while NLOS bias is invisible => future work: vision
    NLOS exclusion, robust float error models, 3DMA. Honest caveats: Medium is
    0-fix (parity by construction); replay protocol noise; n=3.
11. **Conclusion.**

## Tables/figures inventory

- T1: equivalence/cost vs ceres (have).
- T2: frozen-baseline preservation across all builds (have).
- T3: UrbanNav Deep paired BL/VA-v1/VA-v2 n=3 (have; final_deep_3way json).
- T4: diagnosis table (fixed vs float dh; segment before/during/after) (have).
- T5: falsification-chain summary (gates/expiry/dose x mechanism-check x
  outcome) (have except dose verdict).
- T6: FINAL n=3 solo (BL/VA-v4/RF-Cauchy) — DONE (in DRAFT_FULL §7).
- T7: 3-tier metric cross-walk incl. Table V reference — DONE (in DRAFT_FULL §7).
- F1: architecture diagram (graph + where Q_aa flows into LAMBDA).
- F2: two-stage elimination schematic.
- F3: timing vs window size / epoch (have logs).
- F4: error-over-time overlay of a blow-up wave (VA-v2 run2) vs baseline +
  fix-epoch markers (data on disk).
- F5: P_s / std_cycles distribution of accepted subsets (softw logs).

## Integrity statement (for the paper)

GT used only in post-hoc evaluation with the runner's own metric; all
interventions opt-in behind flags with the upstream baseline frozen
(byte-identical effect verified after every build); all decision rules
pre-registered and frozen before runs (PREREG_SOFT_FIX.md,
PREREG_SOFT_WEIGHT.md, with dated addenda for every deviation); all runs
reported including failures.

## TODO before submission

- [ ] VA-v4 n=3 solo verdict (tonight) -> pick title variant, fill T6.
- [ ] Author-eval 3-tier pass over all runs -> T7.
- [ ] VA-v4 Medium n>=1 + GICI-board 3.1/4.1 no-regression.
- [ ] Deep literature pass (BIE, FFRT, GVINS family, NLOS/3DMA).
- [ ] Figures F1-F5.
- [ ] Decide: include VA-v1 (fast-covariance-only) as explicit ablation row.
- [ ] LaTeX port (RAL template).
