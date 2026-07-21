# Claim audit — pre-submission (2026-07-19 evening)

Thesis: real-time exact-in-window joint \(Q_{aa}\) at the optimizer's fixed
linearization. **Paper-1 scientific scope frozen on 2026-07-21.** The unresolved
Deep horizontal tail and NLOS mitigation are limitations and follow-on work,
not blockers for this covariance-method paper.

## Corrected wording (must keep)

| Wrong | Right |
|-------|--------|
| “matches ceres on 99.94%” | “produces validated covariance on 99.94%” (`fast_ok`) |
| “relative trace error ≤1e-3 on 90.1%” | “relative **Frobenius** error ≤1e-3 on 90.1%” (`rel_err` already Frobenius in log) |
| Trace larger ⇒ conservative everywhere | Trace is only a proxy; no directional-dominance claim |
| VA-v4 recommended | Removed |
| Medium in result tables | Withheld because float-path isolation is unavailable |

## Implemented but excluded from Paper-1 claims

- Matrix fields in `[vaar-fast]`: `max_diag_rel`, `min_eig_diff`, `psd_diff`, `gen_eig_*`
- Iso configs + runners: `*_urbannav_iso.yaml`, `run_urbannav_rrr_{bl,va}_iso.py`
- Ceres oracle config + runner: `*_ceres_oracle.yaml`, `run_urbannav_rrr_ceres_oracle.py`
- Identity checker: `scripts/check_float_path_identity.py`
- C/N0/robust-DD/NLOS tail mitigation belongs to the follow-on study.

## Deferred measurements that would enable stronger future claims

1. Loewner/diag/generalized-eigen table: needed for directional dominance.
2. Medium identity: needed to publish a Medium AR-effect table.
3. Ceres oracle e2e: needed for Ceres-equivalent decision/trajectory claims.
4. More independent urban scenes: needed for general urban-accuracy claims.

Paper 1 makes none of those stronger claims. Its bounded claim is covariance
recovery accuracy/runtime plus transparently secondary, mixed UrbanNav impact.
