# Checklist hoàn thiện Công trình 1 — giao cho Cursor thực hiện

**Bối cảnh (đọc trước khi làm gì):** Công trình 1 đã **tái cấu trúc (2026-07-19)** quanh một
luận điểm: *Real-Time Marginal Ambiguity Covariance* (exact-in-window \(Q_{aa}\)).
Chuỗi VA-v2/v3/v4 + robust float = hoãn / báo cáo kỹ thuật — không còn là contribution chính.
Bản thảo + LaTeX đồng bộ luận điểm mới:
- EN: `research/paper/DRAFT_FULL.md` · VN: `research/paper/DRAFT_FULL_VI.md`
- Dàn ý + kiểm kê bảng/hình: `research/paper/DRAFT_OUTLINE.md`
- Nhật ký + số liệu gốc: `research/VISION_AIDED_AR.md` · Bảng tự sinh: `results/research/paper_tables.{md,json}`
- Đăng ký trước: `research/PREREG_SOFT_FIX.md`, `PREREG_SOFT_WEIGHT.md`, `PREREG_ROBUST_FLOAT.md`

Nhánh làm việc: `research/vision-aided-ambiguity-resolution` (worktree `gici_vision_aided_ar`).
KHÔNG đụng nhánh/worktree `research/vision-nlos` (Claude đang làm Công trình 2 ở đó).

---

## ⛔ RÀO CHẮN LIÊM CHÍNH — VI PHẠM = HỎNG CÔNG TRÌNH (đọc kỹ, áp cho mọi mục)

1. **KHÔNG bịa hay chỉnh bất kỳ con số kết quả nào.** Mọi số trong bài phải truy được về
   `research/VISION_AIDED_AR.md`, addendum trong các file PREREG, hoặc `results/research/paper_tables.json`.
   Thiếu số nào → CHẠY lại eval để lấy (`scripts/eval_paper_tables.py`), tuyệt đối không đoán.
2. **KHÔNG nới/tô claim.** Khung bài là **covariance method + validation + e2e honest**.
   Không “proven / unreachable / requires external scene”. Không gọi VA-v4 là recommended.
3. **Overconfidence wording:** "171/177 larger trace; 6 near-tie ≤7.3e-4" **under trace proxy
   only**. KHÔNG viết elementwise “never overconfident”. Runtime phải kèm p95/p99/deadline-miss.
4. **KHÔNG sửa hành vi baseline hay mặc định estimator.** Code Công trình 1 đã đóng băng; mọi cờ
   cải tiến là opt-in mặc-định-tắt. Coi `src/`, `include/` là chỉ-đọc trừ mục 7 (có hướng dẫn riêng).
5. **KHÔNG dùng ground truth để tinh chỉnh** bất cứ thứ gì. GT chỉ ở bước đánh giá hậu kỳ.
6. **EN và VN phải khớp số liệu 100%.** Sửa số ở bản này thì sửa bản kia.
7. Mọi thay đổi thuật toán/tham số (nếu có) phải kèm ghi chú và KHÔNG được phá guard baseline
   (mục 8). Nếu không chắc → dừng và hỏi, đừng tự quyết.

---

## A. Nội dung học thuật (cần người review sau khi Cursor soạn nháp)

- [x] **A1. Related Work (§2) — viết đầy đủ.**
  → EN+VN §2 rewritten; `research/paper/refs.bib` (2026-07-19).
- [x] **A2. Kiểm tính nhất quán EN↔VN**
  → §2 + §7 + abstract/contract sync’d to paper_repro n=3 tables; Medium Δh disclosed.
- [x] **A3. Rà claim trung thực**
  → `research/paper/CLAIM_AUDIT.md` (softened “held at baseline”; n=2→n=3).

## B. Hình minh họa (Cursor sinh được bằng script; dữ liệu đã có)

- [x] **F1. Sơ đồ kiến trúc** → TikZ in `latex/main.tex` (Fig. arch).
- [x] **F2. Sơ đồ khử Schur hai tầng** → TikZ in `latex/main.tex` (Fig. schur).
- [x] **F3. Chi phí thời gian** → `scripts/fig_timing.py` → `fig/F3_timing.pdf` (n=1787, 28 vs 812 ms).
- [x] **F4. Sai số ngang theo thời gian** → `scripts/fig_tail.py` → `fig/F4_tail.pdf` (VA-v4 run3 vs BL).
- [x] **F5. Phân bố P_s/std** → `scripts/fig_dose.py` → `fig/F5_dose.pdf` (n=1286 softw).

## C. Chuyển sang LaTeX để nộp

- [x] **C1. Dựng khung IEEE RAL** → `research/paper/latex/main.tex`.
- [x] **C2. Chuyển nội dung cov-centric + F1–F4 + refs (sparse inv / iSAM2)** → `main.tex`.
- [x] **C3. Biên dịch PDF ≤8 trang** → `latex/main.pdf` (cov-centric letter) via
  user-local `tectonic`. Rebuild: `cd latex && tectonic -X compile main.tex`

## D. Tái lập & vệ sinh nộp bài

- [x] **D1. Tái sinh bảng số sạch** → `results/research/paper_repro/paper_tables.{md,json}`
  synced to `results/research/paper_tables.*` (includes Medium section).
- [x] **D2. README artifact** → `research/paper/README.md`.
- [ ] **D3. Commit** — chỉ khi giáo sư/chủ repo đồng ý (chưa commit).

## E. Tùy chọn tăng độ mạnh (KHÔNG bắt buộc để nộp)

- [ ] **E1. UrbanNav Harsh** — GNSS unzipped; bag/gici_rrr/base chưa sẵn.
- [ ] **E2. Per-ambiguity diagonal overconfidence log** — chưa làm (đụng code).

---

## Thứ tự đề xuất
A1+A2+A3 (nội dung) → F3,F4,F5 (hình có dữ liệu) → F1,F2 (sơ đồ) → C (LaTeX) → D (tái lập/nộp).
E chỉ làm sau khi A–D xong. Mỗi mục xong → tick + ghi 1 dòng kết quả/đường dẫn output ngay dưới mục.

## Trạng thái đóng paper (2026-07-19)

**LaTeX cov-centric `main.pdf` (5 trang) + draft EN/VN v1.0: sẵn sàng review người.
Việc đo còn thiếu (Frobenius, float-path Medium, multi-scene) nằm trong CLAIM_AUDIT.**
Còn lại: (1) review A1/A3 + polish prose nếu cần, (2) commit khi giáo sư đồng ý.
Không mở lại ladder thí nghiệm; E1/E2 không bắt buộc.
