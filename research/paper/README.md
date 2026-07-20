# Paper 1 — Real-Time Marginal Ambiguity Covariance (RAL draft)

**Thesis (2026-07-19 rewrite):** exact-in-window joint \(Q_{aa}\) that is consistent and
online. VA-v2/v3/v4 / robust-float falsification is **deferred** (not the main claim).
Recommended flag for this letter: `ar_use_fast_marginal_covariance` only.

**Not submission-ready yet.** See `SUBMISSION_GAPS.md`.

### Clean re-run (mandatory — discard old numbers)

```bash
# USE THIS — fresh dated tree, cov-centric arms only:
scripts/run_paper_cov_all.sh

# DO NOT use scripts/run_paper_all.sh  (old VA-v4/RF chain)
# DO NOT cite results/research/paper_repro/ or old board logs
```

## Source of truth

| Artifact | Path |
|----------|------|
| Draft (EN) | `DRAFT_FULL.md` |
| Draft (VN) | `DRAFT_FULL_VI.md` |
| Claim audit | `CLAIM_AUDIT.md` |
| Numbers (cov e2e) | `../VISION_AIDED_AR.md` (VA-fast paired) + `[vaar-fast]` log percentiles |
| Numbers (dose chain, deferred) | `../../results/research/paper_repro/paper_tables.md` |
| Pre-registrations (deferred chain) | `../PREREG_*.md` |
| LaTeX | `latex/main.tex` → `latex/main.pdf` |
| BibTeX | `refs.bib` |
| Figures in letter | F1/F2 TikZ, `fig/F3_timing.pdf`, `fig/F4_tail.pdf` (F5 dose demoted) |

## Metric contract (integrity)

- **Primary (method claims):** unaligned raw ENU RMSE vs GT interpolated to solution timestamps.
- **Chi Table V comparison only:** `evo_ape` Sim(3) via `scripts/run_author_eval_urbannav.sh` / T7 cross-walk.
- **No GT in estimator.** VA/RF flags opt-in, default off.
- **Do not invent numbers** — regenerate tables, then edit drafts to match.

## Regenerate tables

```bash
# From validated ad-hoc roots (fast):
python3 scripts/eval_paper_tables.py --today --tier3 \
  --va-log logs/research/gici_board_va/1_1/run.stderr

# Canonical tree written by the master launcher:
python3 scripts/eval_paper_tables.py results/research/paper_repro --tier3 \
  --va-log logs/research/gici_board_va/1_1/run.stderr

# Full unattended regeneration (~4 h, solo protocol):
scripts/run_paper_all.sh results/research/paper_repro
```

Copy `paper_repro/paper_tables.md` into drafts when cells change (draft follows auto tables, never the reverse).

## Regenerate figures

```bash
python3 scripts/fig_timing.py   # → research/paper/fig/F3_timing.pdf
python3 scripts/fig_tail.py     # → research/paper/fig/F4_tail.pdf
python3 scripts/fig_dose.py     # → research/paper/fig/F5_dose.pdf
```

## Build LaTeX

```bash
cd research/paper/latex
# Prefer tectonic (no full TeX Live required):
tectonic -X compile main.tex
# Or classic:
# pdflatex main && bibtex main && pdflatex main && pdflatex main
# Output: main.pdf (currently 3 pages; RAL limit 8)
```

## UrbanNav data (author-faithful)

See `docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md` and `research/AUTHOR_METHODOLOGY.md`.

- Rover: `*.ublox.f9p.splitter.obs`
- Deep base: session `hkkt141g.21o` (5 s), **not** full-day `hkkt141g.rnx`
- Config: `research/config/rtk_imu_camera_rrr_urbannav.yaml` (not dataset `gici_rrr/config.yaml`)
- Disclosed adaptations: `max_age=35`, `relative_frequency=0.01`
