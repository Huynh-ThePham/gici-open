# Runbook — đúng 3 lượt, xác nhận giữa các lượt, rồi DỪNG

**Chế độ:** chuẩn bị xong → chạy tay từng lượt trên máy thật (ngoài sandbox).  
**Không** auto-chain. **Không** dùng kết quả cũ. Sau lượt 3 → dừng.

## OUT_ROOT (cây mới — không tái sử dụng campaign cũ)

```bash
export URBANNAV_DATA_ROOT=/media/theph/Data1/Research/dataset
export GICI_DATA_ROOT=/media/theph/Data1/Research/dataset
export GICI_MAIN=/home/theph/ws_ncs/gici_vision_aided_ar/build/gici_main
cd /home/theph/ws_ncs/gici_vision_aided_ar
OUT=$(cat results/research/paper_cov_LATEST)   # đã trỏ vào cây chuẩn bị mới
echo "$OUT"
```

## Cấm

| Cấm | Lý do |
|-----|--------|
| `results/research/paper_repro/**` | VA-v4 / số cũ |
| `logs/research/gici_board_va/1_1/**` | log không có `max_diag_rel` |
| `paper_cov_20260719_165848` | campaign dở (chỉ có A) — bỏ |
| `scripts/run_paper_all.sh` | launcher bài cũ |
| config `va_urbannav.yaml` / `va4_*` | bootstrap 0.999 / dose |
| Harsh / board 1.2–5.2 trong 3 lượt này | ngoài phạm vi Paper-1 cov |

## Phạm vi dữ liệu (3 lượt)

| Dataset | Path |
|---------|------|
| GICI-board 1.1 | `/media/theph/Data1/Research/dataset/1.1` |
| UrbanNav Medium | `.../UrbanNav-HK-Medium-Urban-1` |
| UrbanNav Deep | `.../UrbanNav-HK-Deep-Urban-1` |

Harsh có trên đĩa nhưng **không** nằm trong 3 lượt này.

---

## Lượt 1 — Board covariance (matrix + runtime)

**Mục tiêu:** bảng T1 mới (Frobenius, diag, Loewner, gen-eig, runtime).

```bash
# Máy rảnh; không có gici_main khác
pgrep gici_main && echo STOP || echo idle
scripts/run_paper_cov_stage.sh A "$OUT"    # build + frozen guard (~15–40 min)
# >>> DỪNG: đọc status/A.txt phải là PASS; xem board_1_1/guard_verdict.txt

scripts/run_paper_cov_stage.sh B "$OUT"    # VA-1.1 benchmark (~45–90 min)
# >>> DỪNG: xác nhận bên dưới trước khi Lượt 2
```

**Checklist xác nhận Lượt 1 (tất cả phải đúng):**

- [ ] `status/A.txt` = `PASS`
- [ ] `status/B.txt` = `PASS`
- [ ] `board_1_1/va/run.stderr` chứa `max_diag_rel=` (không dùng log cũ)
- [ ] `grep -c '\[vaar-fast\]' board_1_1/va/run.stderr` ≥ 1000
- [ ] `board_1_1/matrix_metrics.txt` tồn tại và có Frobenius / gen_eig / >50ms
- [ ] `board_1_1/va/output/solution.txt` ≥ 7100 dòng

```bash
scripts/run_paper_cov_stage.sh status "$OUT"
cat "$OUT/board_1_1/matrix_metrics.txt"
```

---

## Lượt 2 — Medium float-path isolation (GATE)

**Mục tiêu:** 0% NlFix → trajectory hai nhánh gần identical (không confound Deep).

```bash
scripts/run_paper_cov_stage.sh C "$OUT"    # Medium BL iso + VA iso (~40–80 min)
# >>> DỪNG: bắt buộc PASS_TRAJ_IDENTITY
```

**Checklist xác nhận Lượt 2:**

- [ ] `status/C.txt` = `PASS`
- [ ] `iso/identity.txt` có `PASS_FIX_ZERO` **và** `PASS_TRAJ_IDENTITY`
- [ ] median Δh < 1 cm, p95 < 5 cm (in file)
- [ ] solutions: `iso/bl/medium/output/solution.txt` và `iso/va/...` ≥ 25000 dòng

**Nếu FAIL:** dừng toàn bộ campaign. Không sang Lượt 3.

```bash
cat "$OUT/iso/identity.txt"
```

---

## Lượt 3 — Deep confirmatory (shadow + proposed + ceres)

**Mục tiêu:** cùng protocol deterministic; so shadow / proposed / Ceres oracle.  
**Một sequence mỗi arm (n=1)** — không 3-repeat giả lập mẫu độc lập.

```bash
scripts/run_paper_cov_stage.sh D1 "$OUT"   # Deep shadow iso
# >>> xác nhận D1 PASS + đọc deep/shadow/summary.txt

scripts/run_paper_cov_stage.sh D2 "$OUT"   # Deep proposed iso
# >>> xác nhận D2 PASS + đọc deep/proposed/summary.txt

scripts/run_paper_cov_stage.sh D4 "$OUT"   # Deep Ceres oracle (--timeout-s 21600)
# >>> xác nhận D4 PASS + đọc deep/ceres/summary.txt

# Tóm tắt rồi DỪNG (không chạy thêm Harsh / local / n=3)
scripts/run_paper_cov_stage.sh status "$OUT"
python3 - <<PY
import json
from pathlib import Path
out=Path("$OUT")/"deep"
for arm in ["shadow","proposed","ceres"]:
    p=out/arm/"deep"/"metrics.json"
    if not p.exists():
        print(arm, "MISSING"); continue
    m=json.loads(p.read_text())["metrics"]
    print(f"{arm}: h={m['rmse_h_m']:.3f} u={m['rmse_u_m']:.3f} yaw={m['yaw_rmse_deg']:.3f} fix={100*m['fixed_rate']:.1f}% n={m['n_matched']}")
PY
```

**Checklist xác nhận Lượt 3 (rồi DỪNG):**

- [ ] `status/D1.txt`, `D2.txt`, `D4.txt` = `PASS`
- [ ] mỗi `deep/*/deep/output/solution.txt` ≥ 60000 dòng
- [ ] cùng `n_matched` admissibility (≥14000)
- [ ] **không** mở thêm D3 local / Harsh / `run_paper_all` / paper_repro

**Ghi chú:** Stage script yêu cầu D3 trước D4. Để khớp “3 lượt / không local”, dùng lệnh Deep thủ công cho Ceres nếu cần, hoặc chạy D3 rồi **bỏ qua số local** khi viết bài. Khuyến nghị chuẩn bị: đã nới stage D4 nhận D2→D4 (xem `run_paper_cov_stage.sh`).

---

## Ước lượng thời gian (máy rảnh)

| Lượt | Thời gian |
|------|-----------|
| 1 (A+B) | ~1–2 h |
| 2 (C) | ~1 h |
| 3 (D1+D2+D4) | ~3–8 h (Ceres chiếm phần lớn) |

## Sau khi đủ 3 lượt

1. Chỉ lấy số từ `$OUT`  
2. Điền LaTeX / `CLAIM_AUDIT.md`  
3. **Dừng** — không tự chạy thêm  
