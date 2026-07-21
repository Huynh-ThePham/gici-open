# Paper 1 — covariance-freeze manifest

Integrity anchor for the frozen Paper 1 state. After this freeze, Paper 1
receives **presentation fixes only** — no algorithm or numeric changes.
Any change to a file below invalidates the corresponding hash; verify with
`sha256sum -c research/paper/FREEZE_MANIFEST.sha256` from the repo root.

## Provenance

| Field | Value |
|-------|-------|
| Frozen code+paper commit | `228862e5a7064ce6790b175f9c44033ba550956d` |
| Branch | `research/vision-aided-ambiguity-resolution` |
| Tag | `paper1-covariance-freeze` (points at the commit that adds this manifest, one commit after the frozen code+paper commit above) |
| Follow-on (excluded) | untested robust-DD / C-N0 exploration + deep-review hardening + CTest suite — separated out and discarded, not part of this freeze |
| Build type | Release |
| Compiler | /usr/bin/c++ |

## Binary

| Artifact | SHA-256 |
|----------|---------|
| `build/gici_main` | `b6c3ec53598118514259503da6581c213770d852cca40edcf614263f2015781f` |

## Configs (`research/config/*.yaml`)

| Artifact | SHA-256 |
|----------|---------|
| `research/config/rtk_imu_camera_rrr_1_1.yaml` | `e31ab707e3c993dc13313a382e9eddd3228046526ca289978ae4ad32fe2b1999` |
| `research/config/rtk_imu_camera_rrr_rfcva_urbannav_iso.yaml` | `413c6c7a988d433b70d6f8d2b913f8a6308e55c2d9fdd1f3ccfc136aa9e9b227` |
| `research/config/rtk_imu_camera_rrr_rfcva_urbannav.yaml` | `cc2b332716b8ae0f00a178a8cbfc8860eb849d13738367022f01f9d769aaa971` |
| `research/config/rtk_imu_camera_rrr_rfva_urbannav_iso.yaml` | `3e282e7d5f9da345b33f061f266c7741850ed0746f1b6383390d4ef4f4b24acf` |
| `research/config/rtk_imu_camera_rrr_rfva_urbannav.yaml` | `0747a1fc0b44294114cc5e94fda2ab4097b1dd7a1b1c51912cda36b52d9af07e` |
| `research/config/rtk_imu_camera_rrr_urbannav_iso.yaml` | `25159478dc0791b64c9b566f83411b7a81a50b91e561dc22756e257ca944fda6` |
| `research/config/rtk_imu_camera_rrr_urbannav.yaml` | `246f64a60b5197960efb81755f9ef99a8fb93b1e9e7dee234dba7236a5d84418` |
| `research/config/rtk_imu_camera_rrr_va_1_1.yaml` | `22021bdd63ad2a25ab675149dcb15fda835571962858ab8b19c77849c02c9b27` |
| `research/config/rtk_imu_camera_rrr_va3_urbannav.yaml` | `8da0622e1f0b67c42c97be582062e53b3825793ce6babaea136577f053102605` |
| `research/config/rtk_imu_camera_rrr_va4_urbannav_iso.yaml` | `bf968e43a6a6a3195e7ce1fda1157400bd56489ae4c6b077e2d2508724ec2a8b` |
| `research/config/rtk_imu_camera_rrr_va4_urbannav.yaml` | `82e234fdc569064ac536e55c90557e8186feffbcf4766698ed38587c7094f593` |
| `research/config/rtk_imu_camera_rrr_va_urbannav_ceres_oracle.yaml` | `c9f0be81f45f65408a275c98ea98b832ad9df8006c5c5c8f30760a9786489f85` |
| `research/config/rtk_imu_camera_rrr_va_urbannav_iso.yaml` | `b7b2943d2aff90e2d659c9852d85a83ddfdcd9f937ca0c0b239652abd01236f1` |
| `research/config/rtk_imu_camera_rrr_va_urbannav.yaml` | `56dfcc1689f67d0adcac52033d47c2a8928a28c2ed09b5afdfd51592d40318a9` |

## Result summaries (back the paper's cited numbers)

| Artifact | SHA-256 |
|----------|---------|
| `results/research/_archive_20260720/paper_tables.md` | `84c57114aa3b207f96d2a58262428d8c178a5b6464b72b4176f897eb6627f035` |
| `results/research/_archive_20260720/paper_tables.json` | `f4034541f270c2ec53852a392658def34c96891408fe852437b76bcc99798b08` |
| `results/research/_archive_20260720/final_deep_3way_20260718.json` | `03cb7295bcc57dd1b6dac4d40a370bc6b52526829c5cc883c5ab059f82d36875` |
| `results/research/vaar_telemetry_full_20260721_020335/summary.json` | `e1e0472cdd33dca918fe79e2a6a300dcd5917e716abaef1734ea645df1033ecd` |
| `research/VISION_AIDED_AR.md` | `1e4938e02121681361c1a08e41a122164530428404966679d5146b5010a170dc` |
| `logs/research/gici_board_va/1_1/run.stderr` | `ca457fba77e52da866e761842bf54ac124dc54b7e8f7362e71794b7911c6ab85` |

## Supporting evidence (boundary characterization; not cited numerically)

| Artifact | SHA-256 |
|----------|---------|
| `results/research/_archive_20260720/spike_m1_deep.json` | `b2310eb971c26443628ef25e690d6c3c4d063312b3a8b4c6e2a002cf87dc2958` |
| `results/research/_archive_20260720/spike_g1_deep.json` | `26a95ac38086831004ababaf9d39b64e82ee77d5d432edabfc28d9d9840f99ec` |

## Figures

| Artifact | SHA-256 |
|----------|---------|
| `research/paper/fig/F3_timing.pdf` | `41d8bd0bb490b6df86a8485a2949186407f4017e4bf00ad1ac7f042e5937b385` |
| `research/paper/fig/F4_tail.pdf` | `dc05aa7a5f3c84eae9f4347a25f08916f6f317218c21475a1016a3e41a448602` |
| `research/paper/fig/F5_dose.pdf` | `40d2e028d8a3ec738fa5ef039ad94a8c6bae9b47c3a53b54209a28a872d1ed2e` |

## PDF

| Artifact | SHA-256 |
|----------|---------|
| `research/paper/latex/main.pdf` | `466b1afee9e1849d4d22ea1cd562526897cd34584eaea91cc96b6acc313e6412` |
