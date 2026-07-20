# Current State — Consolidated Reference (2026-07-20)

Single source of truth for this research. Read this FIRST to avoid re-confusing things.

---

## 0. THE ANTI-CONFUSION TABLE — three different covariances

The word "covariance" refers to THREE distinct things. Never conflate them again.

| # | Name | Whose | How computed | Verdict | Evidence |
|---|------|-------|-------------|---------|----------|
| 1 | **GNSS-only "shadow" covariance** | **Author (baseline)** | a SEPARATE GNSS-only `RtkEstimator` sub-estimator, decoupled from the joint IMU/camera graph | critique = **DECOUPLED** (ignores IMU/vision info). **NOT called "overconfident".** | Author CODE `src/fusion/rtk_imu_camera_rrr_estimator.cpp:53–62` ("RTK estimator used for ambiguity covariance estimation") + paper [chi2023gici] |
| 2 | **Local-conditioning approximation** | **OUR own rejected attempt** | conditioning-based local approx of Q_aa | **OVERCONFIDENT** → raised fix rate → worse Deep | OUR data: Deep 6.75 ± 2.98 m, fix ~17% (draft Table) |
| 3 | **Joint marginal covariance** | **OUR contribution** | `Graph::getMarginalAmbiguityCovariance` — Q_aa = [H_active⁻¹]_aa from the FULL joint graph incl. marginalization prior | consistent (≈ exact ceres, 90% within 1e-3) + real-time (28 ms vs 812 ms, ~29×) | OUR data (open-sky 1.1) |

**Honesty rules locked in:**
- We say about the AUTHOR only what their CODE + PAPER show: "shadow covariance, decoupled." We do NOT call the author overconfident.
- "Overconfident" applies ONLY to OUR OWN local-conditioning variant, backed by OUR data.
- (Chat-history note: an earlier verbal "baseline overconfident" was a CONFLATION of #1 and #2 — retracted.)

---

## 1. Contribution boundary (do not blur)

- **MAIN = the method (covariance #3):** consistent + real-time joint AR covariance. Proof lives on **open-sky (1.1)**. It is a METHOD-PROPERTY contribution (consistency + speed), **NOT** "better positioning accuracy."
- **SECONDARY = the boundary result:** pre-registered falsification chain shows the deep-urban horizontal tail is not fixable in-estimator. Strong/honest, but must not replace the main contribution.
- **Hard honesty constraint:** the method does NOT improve horizontal accuracy on deep-urban. Baseline wins horizontal (ISO bake-off: bl 2.316 m best; va4 2.734 m). va4 DOES win vertical −57% / yaw / fix ×3.6 — a disclosed multi-dim trade-off, not a headline.

---

## 2. Boundary evidence — FOUR independent falsifications, one root cause

Deep-urban horizontal tail is **self-consistent specular multipath on strong-signal LOS satellites**, carrier-domain, invisible to every single-station GNSS-internal observable:

| Falsification | Indicator | corr with h_err | Source |
|---|---|---|---|
| Decision-layer chain | gate→soft→dose→robust | — (all fail) | pre-registered runs |
| **M1** skyline-NLOS | NLOS count | **−0.012** | `scripts/spike_m1_los_nlos.py`, `results/research/spike_m1_deep.json` |
| **G1** classical weighting | code-MP / SNR | **< 0.15**; high-err SNR = STRONG | `scripts/spike_g1_mp_snr.py`, `results/research/spike_g1_deep.json` |
| (partial) NEES vertical | position σ vs GT | vertical consistent; horiz eval-contaminated | `results/research/nees/` |

→ No cheap fix (classical GNSS-internal OR skyline-NLOS-exclusion) works — quantified. A real horizontal win needs external-geometry multipath correction (3DMA / LiDAR ray-tracing, no-AI) — HARD, months, future work.

---

## 3. NEES / integrity test — status (honest)

- Goal: prove covariance #3 is statistically consistent (calibrated), not just numerically ≈ exact.
- $GPESD NMEA (with `compute_covariance:true`) gives per-epoch position σ → NEES-able.
- **Position NEES is eval-alignment-sensitive** (raw ENU on 1.1 = 32 cm vs Sim(3)-aligned 2.9 cm → ~30 cm is FRAME misalignment, NOT estimator error). Horizontal result INVALID as-is; **vertical axis (clean) shows CONSISTENT covariance** (E[z²]=0.83, 72% @1σ vs 68% target) — positive signal.
- **The clean test of OUR method = Q_aa success-rate calibration** (predicted P_s vs empirical fix correctness) — direct, cheap, frame-independent. NOT YET RUN.
- Position integrity needs the author's Sim(3)-aligned eval to be valid. TODO / open decision.

---

## 4. Infrastructure — clean & ready

- **Datasets** at `/media/theph/Data1/Research/dataset/UrbanNav/UrbanNav-HK-{Deep,Medium,Harsh}-Urban-1` (extra `UrbanNav/` dir level). Source-only re-prep 2026-07-19; `gici_rrr/` (imu.bin.txt + camera.bin) regenerated via `scripts/extract_urbannav_gici_rrr.py` (rosbags lib, no ROS build). Skymask verified (all 3, covers trajectory).
- **Base station:** all 3 = HKKT 5 s session from HK SatRef open archive `rinex.geodetic.gov.hk`. `scripts/extract`... See `dataset/UrbanNav/BASE_STATION_PREPARATION.md`. Harsh needed **flag≥2 event-block stripping** (concatenated hourly RINEX) — segfault ROOT-CAUSED (RTKLIB `readrnxobsb` on mid-stream flag-4 event) + FIXED (data-level, no RTKLIB change) + verified (32559 GPGGA clean).
- **Runner:** `scripts/run_urbannav_rrr_baseline.py` (+ VA variants import it). Hardened watchdog (hang-detection). Eval Q≤2 gate + [453,2764]s window for Harsh (`gt_q_max=2`).
- **Deterministic configs (ISO):** `num_threads:1 + max_solver_time:1e9` for reproducible n=1 (the 0.04/4-thread real-time configs vary 2.4–3.8 m run-to-run). ISO runners for bl/va/va3/va4/rfva/rfcva.

### Canonical result dirs (this session)
- `results/research/va_bakeoff_iso_20260720/` — Deep bake-off ISO (bl 2.316 / va 3.177 / va3 9.755 broken / **va4 2.734** / rfva crash / rfcva 3.438 m).
- `results/research/base5s_compare_20260719/` — 5 s baselines (Medium 2.26, Deep 2.14–3.79 nondeterministic-caveat, Harsh_iso_fix 7.569 clean).
- `results/research/spike_m1_deep.json`, `spike_g1_deep.json` — boundary falsifications.
- `results/research/nees/` — NEES attempt (vertical-consistent, horiz eval-contaminated).

---

## 5. Open decisions (for the user)
1. Integrity proof path: (A′) Sim(3)-aligned position NEES, or (B) Q_aa success-rate calibration (recommended — direct/clean).
2. Verify [chi2023gici] paper text for the exact author wording on the shadow covariance.
3. Paper: SPLIT (method + boundary now) vs merge-with-3DMA (months, hard). Recommendation: SPLIT; 3DMA future work.
