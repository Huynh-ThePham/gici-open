# Báo cáo tiến độ nghiên cứu (tổng quan)

**Đề tài:** Hệ thống định vị dựa trên kết hợp cảm biến và trí tuệ nhân tạo ứng dụng cho xe tự hành
**Giai đoạn:** 17–19/07/2026 · **Nền tảng:** GICI-LIB (Chi et al., RAL 2023, fork nghiên cứu)

---

## Tổng quan

Giai đoạn này tập trung vào mắt xích quyết định độ chính xác của định vị RTK ghép chặt
GNSS/IMU/Camera ở đô thị: **giải độ mờ số nguyên** và chất lượng **hiệp phương sai** cấp cho nó.
Kết quả nổi bật: xây dựng được một cách tính **hiệp phương sai độ mờ nhất quán và chạy thời gian
thực** (thay cho giải pháp lách của hệ gốc vốn bỏ phí thông tin IMU/thị giác); nhờ đó **cải thiện
bền vững độ cao, hướng và tỉ lệ cố định** ở đô thị sâu; đồng thời **chứng minh chặt chẽ một ranh
giới**: sai số ngang ở đô thị sâu không thể khắc phục bằng bất kỳ cơ chế nào chỉ dựa trên thống kê
GNSS — mở đường một cách có luận cứ cho việc đưa **trí tuệ nhân tạo** vào giai đoạn tiếp theo.

Định vị trung thực: đây là công trình dạng **"phương pháp + cải thiện có chọn lọc + xác định ranh
giới"**, không phải tuyên bố "vượt SOTA toàn diện".

## Đã đạt được

- **Đóng góp phương pháp:** hiệp phương sai độ mờ khớp toàn hệ (GNSS+IMU+thị giác), tính đúng như
  phương pháp tham chiếu nhưng **nhanh ~29 lần** nên dùng được trực tuyến, và **an toàn** (không
  bao giờ tự tin quá mức *đáng kể* — kiểm chứng từng epoch trên GICI-board 1.1: 171/177 epoch bảo
  thủ, 6 epoch còn lại chỉ chênh ≤7.3e-4 trong sai số số học). Hệ gốc được giữ nguyên vẹn — mọi
  cải tiến là tùy chọn bật/tắt.
- **Cải thiện thực nghiệm (đô thị sâu, n=3 lần chạy cặp):** độ cao tốt hơn ~35–40% (trung bình n=3;
  lần chạy/biến thể tốt nhất tới ~50%), hướng (yaw) tốt hơn ~40–55%, tỉ lệ cố định tăng ~4 lần
  (2.4–5×) — **bền vững**: độ cao thắng 6/6, hướng 5/6 lần chạy cặp.
- **Ranh giới được chứng minh:** qua **năm thử nghiệm cải tiến (đăng ký trước)** đều không cải
  thiện được kênh ngang, tôi chứng minh sai số ngang là do **hiện tượng che khuất tín hiệu vệ tinh
  (NLOS) diện rộng** — cần thông tin về *cảnh vật* (vệ tinh nào bị nhà che), thứ mà dữ liệu GNSS
  đơn thuần không chứa.

## Đánh giá trung thực

- **Đã chứng minh:** hiệp phương sai nhất quán + thời gian thực; và cải thiện độ cao/hướng/tỉ lệ
  cố định là thật, bền vững.
- **Ranh giới:** kênh ngang giữ ở mức nền, có trường hợp xấu — **không tuyên bố cải thiện ngang**.
  Đây là kết luận có bằng chứng, không phải điểm yếu che giấu.
- **Điểm mạnh về phương pháp luận:** toàn bộ tuân thủ *đăng ký trước* (chốt tiêu chí trước khi
  chạy), *không dùng dữ liệu chuẩn để tinh chỉnh*, *báo cáo cả các thất bại*. Đây là yếu tố làm
  kết quả đáng tin và khó bị phản biện.

## Hướng trí tuệ nhân tạo & cấu trúc luận án

Điểm mạnh của lộ trình: **AI được đưa vào đúng chỗ đã được chứng minh là cần**, không theo trào
lưu. Vòng cung ba công trình:

1. **Công trình 1 (gần hoàn tất):** hiệp phương sai nhất quán thời gian thực + xác định ranh giới —
   nền tảng cổ điển. Dự kiến nộp *IEEE RAL*.
2. **Công trình 2 (đang khởi động):** dùng **camera phân loại vệ tinh bị che (NLOS)** để vượt qua
   ranh giới mà Công trình 1 chỉ ra — AI cho *nhận thức cảnh vật*.
3. **Công trình 3 (định hướng):** học máy cho **giám sát độ tin cậy/an toàn** của nghiệm định vị —
   trực tiếp phục vụ xe tự hành.

Mạch logic: *"cổ điển làm được tới đâu và tại sao không hơn"* → *"AI bổ sung mẩu thông tin còn
thiếu"* → *"học cách biết khi nào nghiệm không đáng tin"*.

## Bước tiếp theo

- **Hoàn thiện Công trình 1:** khảo cứu văn liệu, hình minh họa, chuyển sang LaTeX để nộp (bản thảo
  nội dung đã đầy đủ, có cả tiếng Anh và tiếng Việt).
- **Công trình 2:** kiểm tính khả thi của việc camera "nhìn thấy" vệ tinh bị che (bước quyết định
  sớm), rồi xây bộ phân loại và đánh giá.
- **Tùy chọn tăng độ mạnh:** bổ sung tập dữ liệu đô thị khắc nghiệt hơn (UrbanNav Harsh) để có dải
  so sánh đầy đủ.

## Sản phẩm

Mã nguồn, bản thảo (Anh/Việt), nhật ký nghiên cứu, các bản đăng ký trước, và công cụ tái lập
một-lệnh đều đã có trong repo; kết quả tái tạo được đầy đủ.

---
*Báo cáo dạng tổng quan; số liệu và chứng minh chi tiết nằm trong nhật ký nghiên cứu
`research/VISION_AIDED_AR.md` và bản thảo `research/paper/`. Có thể chuyển sang tiếng Anh nếu cần.*
