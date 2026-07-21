# Hiệp phương sai độ mờ biên thời gian thực cho đồ thị nhân tố GNSS–thị giác–quán tính ghép chặt

*Bản thảo v1.0 — 2026-07-19. Mục tiêu: IEEE RAL. Tập trung một luận điểm: hiệp phương
sai độ mờ biên exact-in-window, nhất quán và chạy online. Chuỗi VA-v2/v3/v4 và robust
float được tách / hoãn — không claim trong bài này. Số liệu từ
`research/VISION_AIDED_AR.md` và log `[vaar-fast]`; không bịa số.*

LaTeX (EN): `research/paper/latex/main.tex` → `main.pdf`.

---

## Tóm tắt

AR pha sóng mang trong bộ ước lượng GNSS/IMU/thị giác ghép chặt cần hiệp phương sai
độ mờ float \(Q_{aa}\). Các nền tảng mã mở thường dùng hiệp phương sai “bóng” chỉ-GNSS
vì tạo hiệp phương sai toàn cửa sổ online bị coi là quá đắt — bỏ đúng thông tin IMU và
thị giác mà ghép chặt nhằm cung cấp. Chúng tôi đưa ra biên exact-in-window
\(Q_{aa}=[H_{\mathrm{active}}^{-1}]_{aa}\) bằng khử Schur thưa hai tầng trên cửa sổ
GNSS–IMU–thị giác (kèm prior biên), kèm cổng tự kiểm tra số và fallback sang bóng.

Trên 1.787 epoch AR trời mở: khớp `ceres::Covariance` 99,94% epoch (90,1% sai số trace
tương đối \(\le10^{-3}\)); trong 177 bất đồng lớn hơn, 171 có trace lớn hơn (bảo thủ),
6 là near-tie (thâm hụt tệ nhất \(7{,}3\times10^{-4}\)). Thời gian trung bình 28 ms so
với 812 ms Ceres (~29×); trung vị 27 ms, p95 49 ms, p99 59 ms; 4,3% epoch vượt ngân sách
50 ms (max 97,5 ms). End-to-end, thay bóng cải thiện sai số đứng, yaw và tỷ lệ fix trên
UrbanNav Deep; ngang vẫn hỗn hợp; xấp xỉ local-conditioning tự tin quá mức thì có hại.
Không claim lớp quyết định nào chữa được lỗi ngang đô thị sâu.

---

## 1. Mở đầu

Ba đóng góp (cùng cấp — đều về \(Q_{aa}\)):

1. Thuật toán trích \(Q_{aa}\) exact-in-window bằng khử hai tầng, gồm prior biên, không
   nghịch đảo toàn \(H\).
2. Cổng tự kiểm tra số + abstention có kiểu + fallback bóng.
3. Đánh giá cấp hiệp phương sai và end-to-end so với bóng, local conditioning, và Ceres
   trên trời mở và UrbanNav Medium/Deep.

Chuỗi cổng / hết hạn / dose \(P_s\) / Tukey–Cauchy **không** thuộc đóng góp thuật toán
của thư này.

---

## 2–4. Related work, bài toán, phương pháp

Related work bổ sung: selected / sparse inversion, iSAM2 / Bayes tree, Ceres covariance
— ngoài LAMBDA/BIE và GNSS–VIO.

\(H\) cửa sổ + prior; cần \(Q_{aa}=[H^{-1}]_{aa}\).
R1 exact-in-window; R2 chi phí theo cửa sổ + fallback; R3 ưu tiên bất định lớn hơn
khi số học bất đồng.

Stage 1: Schur landmark \(3\times3\) rank-truncated.
Stage 2: Jacobi + LDLT + refinement gate; thất bại → bóng.

---

## 5–6. Kiểm chứng số và runtime

| Chỉ số | Giá trị |
|---|---|
| Usable | 99,94% / 1787 (1 abstention) |
| Rel. trace ≤ 1e-3 | 90,1% usable |
| Bất đồng > 1e-3 | 177 (171 lớn hơn / 6 near-tie) |
| Proposed ms mean/med/p95/p99/max | 28,0 / 26,7 / 49,1 / 58,8 / 97,5 |
| > 50 ms | 4,3% |
| Ceres tương ứng | 811,9 / 758 / 1573 / 1892 / 2714; >50 ms 99,6% |

**Lưu ý:** proxy online là **trace** — chưa đủ để nói “không overconfident theo từng
ambiguity / hướng”.

Trời mở: 0,028991 m / 0,4727°; 869/1775 fix vs 876/1775 baseline.

---

## 7. Đánh giá end-to-end

Ablation Deep ngang: bóng ~2,8–3,0 m; local-conditioning **6,75±2,98 m** (hại); đề xuất
**3,40±0,78 m** với fix 8,8%. Ceres e2e UrbanNav chưa chạy (chi phí).

Paired \(n=3\): Deep đứng/yaw thắng 3/3; fix ~5×; ngang hỗn hợp. Medium 0% fix cả hai;
\(\Delta h\) nhỏ nghiêng về đề xuất — là khác biệt **đường float**, không claim độ chính
xác Medium.

VA-v4 / chuỗi falsification: tách sang báo cáo kỹ thuật; **không** gọi VA-v4 là cấu hình
khuyến nghị của bài này (khi bật dose, μ ngang solo có thể xấu hơn baseline).

---

## 8. Giới hạn

- Chỉ so trace; thiếu Frobenius / diagonal / trị riêng suy rộng
- 4,3% trễ deadline 50 ms
- Fallback UrbanNav chưa bảng hóa
- Medium 0-fix chưa cô lập float-path
- \(n=3\) cùng một trajectory; chưa Harsh / thành phố khác
- Replay phụ thuộc tải máy = hạn chế tái lập, không phải contribution
- Không dùng “proven” / “unreachable” / “bắt buộc thông tin cảnh”

---

## 9. Kết luận

Hiệp phương sai độ mờ biên exact-in-window chạy được online (~29× so Ceres), khớp phần
lớn epoch theo proxy trace, giữ AR trời mở, tránh sụp local-conditioning trên Deep, cải
thiện đứng/yaw/fix — không bảo đảm ngang đô thị sâu. Bước đo tiếp: kiểm chứng cả ma trận,
cứng deadline, cô lập float-path, thêm môi trường. Lớp quyết định cho đuôi ngang là việc
riêng.

---

## Hợp đồng claim

- Metric chính: raw ENU. Chi Table V: chỉ Sim(3).
- An toàn cov: conservative-or-tied **theo trace**; chưa chứng minh từng phần tử.
- Real-time: luôn kèm p95/p99/max và tỷ lệ vượt 50 ms.
- Cờ khuyến nghị bài này: chỉ `ar_use_fast_marginal_covariance`.
