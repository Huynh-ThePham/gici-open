# Vision-based NLOS-aware GNSS float for deep-urban RTK — scope

*Created 2026-07-19. Follow-on to the consistent-covariance/decision-layer study
(research/VISION_AIDED_AR.md). Motivated directly by that study's proven boundary: the
deep-urban horizontal error is a MAJORITY-affecting NLOS bias, invisible to every
in-estimator mechanism operating on GNSS statistics (gates, constraint expiry,
decision-confidence dose, per-residual float M-estimation). The only remaining
lever is external scene information that identifies WHICH satellites are blocked.*

## Thesis

The camera already onboard a tightly-coupled RTK/IMU/vision system can be used
not only to aid the pose (Paper 1) but to **classify satellite lines-of-sight as
LOS/NLOS from scene geometry**, and this classification, applied to the GNSS
float estimate upstream of ambiguity resolution, removes the horizontal tail that
Paper 1 proved is out of reach of GNSS-statistics-only methods — while retaining
Paper 1's vertical/yaw/availability gains (VA-v4 kept above it).

## Why this is a distinct contribution (not a Paper 1 addendum)

- Requires a new information source (scene structure → per-satellite visibility),
  not a reweighting of existing residuals. Paper 1 exhausted the latter.
- It is a system: camera–GNSS spatial + temporal registration to project
  satellite directions into the image, a sky/building classifier, and a fusion
  rule. Each is a design axis with its own validation.
- Clean separation keeps both papers honest: Paper 1 = "what the covariance +
  decision layer can and cannot do"; Paper 2 = "adding vision NLOS crosses the
  boundary Paper 1 drew."

## MVP (minimum to close ONE paper) — four blocks

1. **Per-satellite LOS/NLOS label from the camera.**
   - Inputs available in-repo: satellite azimuth/elevation (already computed in
     `gnss_common` for elevation gating), camera intrinsics/extrinsics (ZED2 left,
     `T_B_C` in the config), IMU-propagated attitude (gives camera orientation in
     the local frame each epoch).
   - Method (MVP, cheapest first): project each satellite direction (az/el) into
     the image plane via the current camera pose; classify the pixel region as
     sky vs building. Two candidate classifiers, report both:
     (a) **geometric skyline** — a light sky-segmentation (brightness/gradient or a
         small pretrained segmentation net run at low rate) → a per-azimuth
         elevation horizon; satellite is NLOS if its elevation < horizon(azimuth).
     (b) **semantic** — an off-the-shelf sky/building segmentation network.
   - Caveat carried from Paper 1's honest style: the ZED2 is FORWARD-facing with a
     limited FOV, so only satellites roughly ahead are directly observable. For
     out-of-FOV satellites, fall back to elevation-only (no vision label). This
     partial-observability is itself a finding to characterize, not hide.

2. **Apply the label to the float, upstream of AR.**
   - NLOS-labeled satellites: down-weight (inflate measurement variance by a
     principled factor tied to expected multipath excess, or reject if confidently
     NLOS). This is where Paper 1's finding bites: because the bias is
     majority-affecting, *removing/deweighting the identified NLOS subset* is the
     move a per-residual robust loss could NOT make (it could not tell which were
     bad); the camera tells it which.
   - Keep the hard-threshold FDE and the consistent covariance path unchanged.

3. **Keep VA-v4 above it.** Consistent real-time covariance + decision-confidence
   fix weighting (Paper 1's recommended config) runs unchanged on top of the
   NLOS-cleaned float. The hypothesis is compositional: cleaner float → AR fixes
   anchored to unbiased states → no tail; VA-v4 still delivers u/yaw/fix.

4. **Pre-registered Deep n=3 evaluation**, same protocol/integrity spine as
   Paper 1 (raw ENU primary + ATE SE(3)/APE Sim(3) crosswalk; solo or paired;
   frozen bins). Target result: cut the horizontal tail (no run with d_h > +1.0 m
   vs baseline) AND retain u/yaw/fix. Attribution arms: vision-NLOS-only (no
   VA-v4) and VA-v4-only (Paper 1) to separate contributions.

## Concrete first steps (when Paper 1 is submitted)

- S1. Verify satellite az/el are accessible per-epoch at the estimator level and
  can be transformed into the camera frame (extrinsics + IMU attitude). Prototype
  the projection and overlay satellite marks on a few UrbanNav Deep frames to
  eyeball plausibility (sanity, GT-free).
- S2. Implement the cheapest classifier (geometric skyline) first; log a per-epoch
  `[nlos] n_sat=.. n_labeled=.. n_nlos=..` mechanism check.
- S3. Prereg PAPER2 H1 + frozen bins BEFORE any Deep accuracy run.
- S4. Single exploratory Deep run (as in Paper 1's "one run first" discipline) to
  confirm the classifier engages and nothing crashes; then n=3.

## Risks / honest priors

- Forward-FOV limits vision coverage of the sky → partial NLOS labeling →
  partial tail reduction (still publishable, and honest about coverage).
- Skyline classifier failure in glare/rain/tunnels → need a confidence gate that
  falls back to elevation-only.
- Camera–GNSS temporal offset (the `td` state) matters for projection accuracy at
  vehicle speed — ties into the separately-scoped online-temporal-calibration
  direction; MVP can use a fixed offset and note the sensitivity.
- If even correct NLOS removal leaves a tail (e.g. diffraction/multipath from LOS
  satellites), that is the next boundary — but it would be a NEW finding beyond
  Paper 1.

## Relation to the other endorsed directions (kept separate)

- Online temporal calibration (`td` state) — a distinct paper; feeds Paper 2's
  projection accuracy but is not required for the MVP (fixed offset acceptable).
- Continuous-time FGO — distinct paper; orthogonal to NLOS.
