# Real-Time Marginal Ambiguity Covariance for Tightly Coupled GNSS–Visual–Inertial Factor Graphs

*Draft v1.0 — 2026-07-19. Target: IEEE RAL. Refocused on a single thesis: exact-in-window
joint ambiguity covariance that is consistent and online. Decision-layer VA-v2/v3/v4 and
robust-float falsification are deferred (not claimed here). Numbers from
`research/VISION_AIDED_AR.md` and `[vaar-fast]` logs; no invented stats.*

Companion LaTeX: `research/paper/latex/main.tex` → `main.pdf`.

---

## Abstract

Carrier-phase ambiguity resolution (AR) in tightly coupled GNSS/IMU/vision estimators
consumes a float ambiguity covariance \(Q_{aa}\). Open platforms often substitute a
GNSS-only “shadow” covariance because forming the joint sliding-window covariance
online is considered too expensive—discarding the IMU and visual information that tight
coupling is meant to provide. We present an exact-in-window marginal
\(Q_{aa}=[H_{\mathrm{active}}^{-1}]_{aa}\) obtained by two-stage sparse Schur
elimination over the active GNSS–IMU–vision window, including the marginalization prior,
with a numerical self-validation gate and shadow fallback.

On 1,787 open-sky AR epochs the method matches `ceres::Covariance` on 99.94% of epochs
(90.1% within \(10^{-3}\) relative trace error); among 177 larger disagreements, 171 have
larger trace (conservative) and 6 are numerical near-ties (worst deficit
\(7.3\times10^{-4}\)). Mean runtime is 28 ms versus 812 ms for Ceres (~29×), with median
27 ms, p95 49 ms, p99 59 ms, and 4.3% of epochs above a 50 ms budget (max 97.5 ms).
End-to-end, replacing the shadow covariance improves vertical accuracy, yaw, and fix
availability on UrbanNav Deep relative to the float-heavy baseline, while horizontal
accuracy remains mixed; an overconfident local-conditioning approximation is harmful.
We do not claim a decision-layer fix for deep-urban horizontal error.

---

## 1. Introduction

Tightly coupled GNSS/IMU/vision estimators resolve carrier-phase integers using a float
ambiguity covariance fed to LAMBDA-type search and acceptance tests. In GICI-LIB
[chi2023gici], AR consumes a GNSS-only shadow covariance because a full-graph
`ceres::Covariance` query is too slow online (mean 812 ms on filled-window open-sky
graphs; 99.6% of epochs exceed 50 ms). The shadow path discards IMU and visual
information at the moment AR needs it most.

Cheaper alternatives that condition on neighboring states without the Schur complement
can be overconfident: on UrbanNav Deep, a local-conditioning variant raised fix rate
while degrading horizontal RMSE from \(2.77\pm0.44\) m to \(6.75\pm2.98\) m. AR therefore
needs a covariance that is both consistent with the joint window and affordable every
epoch.

**Contributions (same level — all about \(Q_{aa}\)):**

1. An algorithm for exact-in-window \(Q_{aa}=[H_{\mathrm{active}}^{-1}]_{aa}\) via
   two-stage elimination, including the marginalization prior, without inverting the
   full information matrix.
2. A numerical self-validation gate with typed abstention and shadow fallback.
3. Covariance-level and end-to-end evaluation against shadow, local conditioning, and
   `ceres::Covariance` on open-sky and UrbanNav Medium/Deep.

Decision-layer interventions (gates, expiry, \(P_s\) dose, robust float) are **not**
claimed here; see Sec. 8 / deferred report.

---

## 2. Related Work

**Integer estimation.** LAMBDA / MLAMBDA, ratio / FFRT, PAR, \(P_s\), BIE — standard.
We do not propose a new integer estimator.

**GNSS/IMU/vision fusion.** VINS, OKVIS, GVINS, GICI-LIB. GICI’s AR shadow covariance
is the interface gap we close.

**Covariance recovery.** Sparse / selected inversion [erisman1975sparse, li2008selected];
iSAM2 / Bayes tree covariance extraction [kaess2012isam2, dellaert2017factor]; Ceres
sparse-QR covariance [agarwal2023ceres]. Our method is a domain-structured elimination
for the ambiguity block of an RRR window with an AR-oriented abstention policy.

**Urban GNSS.** Shadow matching, 3DMA, UrbanNav-HK. Used as stress tests, not as a new
NLOS method.

---

## 3. Problem Formulation

Active-window information \(H=\sum_r J_r^\top J_r\) (Triggs Jacobians) plus prior
\(\Lambda\). Partition \((a,\ell,n)\). Need \(Q_{aa}=[H^{-1}]_{aa}\).

Requirements: (R1) exact in-window vs Ceres up to linearization/numerics; (R2) window-
bounded cost with fallback; (R3) prefer larger uncertainty over silent overconfidence.

---

## 4. Proposed Method

Assemble \(H\) including prior via read-only \(\Lambda\) accessor.

Stage 1: per-landmark rank-truncated Schur (\(3\times3\)).
Stage 2: Jacobi-equilibrate nuisance; dense LDLT + iterative refinement; gate on
refinement residual vs smallest-eigenvalue scale; else return shadow \(Q\).

Opt-in estimator `rtk_imu_camera_rrr_va`. Disclosed UrbanNav file-mode adaptations:
`max_age=35`, `relative_frequency=0.01` (not GT-tuned).

---

## 5–6. Numerical Validation and Runtime

| Metric | Value |
|---|---|
| Usable (fast path) | 99.94% of 1787 (1 abstention) |
| Rel. trace err ≤ 1e-3 | 90.1% of usable |
| Disagreements > 1e-3 | 177 (171 larger / 6 near-tie ≤ 7.3e-4) |
| Proposed ms (mean/med/p95/p99/max) | 28.0 / 26.7 / 49.1 / 58.8 / 97.5 |
| > 50 ms | 4.3% (76/1787) |
| Ceres ms (mean/med/p95/p99/max) | 811.9 / 758.1 / 1573 / 1892 / 2714 |
| Ceres > 50 ms | 99.6% |

**Caveat:** online metric is **trace**. Frobenius, per-diagonal, and generalized
eigenvalues are required before stronger elementwise non-overconfidence claims.

Open-sky e2e: 0.028991 m / 0.4727°, 869/1775 fixed vs baseline 876/1775 (in frozen band).

---

## 7. End-to-End AR Evaluation

**Ablation (Deep horizontal):**

| Covariance | Deep \(h\) (m) | Fix |
|---|---|---|
| Shadow | \(2.77\pm0.44\) / \(3.03\pm0.68\)† | ~1.3–1.7% |
| Local conditioning | \(6.75\pm2.98\) | ~17% |
| Proposed | \(3.40\pm0.78\) | \(8.8\pm1.2\%\) |
| Ceres e2e UrbanNav | not run (cost) | — |

†Contemporaneous baselines differ by study wave; see LaTeX table note.

**Paired proposed vs shadow (\(n=3\)):** Deep vertical/yaw win 3/3; fix ~5×; horizontal
mixed (worse 2/3). Medium 0% fix both arms; \(h\) \(2.85\pm0.10\) vs \(2.99\pm0.16\) —
float-path difference, not an accuracy claim.

**Interpretation:** joint \(Q_{aa}\) increases fix willingness when IMU/vision support
the float; it does not guarantee robust horizontal positioning under biased NLOS.
Example tail plot (F4) is suggestive of delayed constraint effects, not a controlled
causal proof.

**Deferred:** VA-v2/v3/v4, Tukey/Cauchy chain — technical report / follow-on. VA-v4 is
**not** recommended as the main contribution (horizontal μ worse than baseline on
confirmatory solo tables when dose is enabled).

---

## 8. Limitations

- Trace-only safety proxy
- 4.3% deadline misses at 50 ms
- Fallback rate on UrbanNav not tabulated
- Medium 0-fix float-path not fully isolated
- \(n=3\) repeats of one Deep trajectory; no Harsh / second city
- Replay load sensitivity (protocol issue, not a contribution)
- Prior-poisoning hypothesis needs stronger causal experiments
- Avoid words: “proven”, “unreachable”, “requires external scene information”

---

## 9. Conclusion

Exact-in-window marginal ambiguity covariance is computable online (~29× vs Ceres on
filled windows), agrees on the large majority of epochs under a trace metric, preserves
open-sky AR, avoids local-conditioning collapse on Deep, and improves vertical / yaw /
fix availability—without a horizontal-accuracy guarantee in deep urban. Next:
matrix-level validation, deadline hardening, float-path isolation, multi-scene urban
eval. Decision-layer horizontal mitigation is separate work.

---

## Metric / claim contract

- Primary method metric: raw ENU (unaligned).
- Chi Table V: Sim(3) only.
- Covariance safety wording: conservative-or-tied **under trace proxy**; not elementwise proven.
- Real-time wording: report mean **and** p95/p99/max/deadline-miss; not “hard real-time”.
- Recommended flag for this paper: `ar_use_fast_marginal_covariance` only.
