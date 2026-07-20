# Claim audit — pre-submission (2026-07-19 evening)

Thesis: exact-in-window joint \(Q_{aa}\). **Not submission-ready** until
`SUBMISSION_GAPS.md` items 1–3 are green.

## Corrected wording (must keep)

| Wrong | Right |
|-------|--------|
| “matches ceres on 99.94%” | “produces validated covariance on 99.94%” (`fast_ok`) |
| “relative trace error ≤1e-3 on 90.1%” | “relative **Frobenius** error ≤1e-3 on 90.1%” (`rel_err` already Frobenius in log) |
| Trace larger ⇒ conservative everywhere | Trace proxy only; need Loewner / gen-eig |
| VA-v4 recommended | Removed |
| Medium in result tables | Withheld until float-path isolation PASS |

## Implemented toward gaps (code/config; runs pending)

- Matrix fields in `[vaar-fast]`: `max_diag_rel`, `min_eig_diff`, `psd_diff`, `gen_eig_*`
- Iso configs + runners: `*_urbannav_iso.yaml`, `run_urbannav_rrr_{bl,va}_iso.py`
- Ceres oracle config + runner: `*_ceres_oracle.yaml`, `run_urbannav_rrr_ceres_oracle.py`
- Identity checker: `scripts/check_float_path_identity.py`
- Plan: `research/paper/SUBMISSION_GAPS.md`

## Still required before submit

1. Rebuild binary; re-run board 1.1 benchmark; fill Loewner/diag/gen-eig table
2. Medium iso PASS_TRAJ_IDENTITY
3. Ceres oracle e2e (full or open-sky + Deep segment)
4. Deterministic confirmatory Deep after (2)
