# Hiệp phương sai độ mờ nhất quán, thời gian thực cho RTK GNSS/IMU/Camera — và giới hạn của kiểm định độ mờ trong bộ ước lượng ở đô thị sâu

*Bản thảo v0.3 — 2026-07-19. Đích nộp: IEEE RAL. Mọi số liệu là số cuối, truy vết được tới
research/VISION_AIDED_AR.md và ba tệp PREREG_*.md kèm addendum có ngày giờ; năm can thiệp
đã đăng ký trước đều đã hoàn tất (gates, expiry, dose, robust-float cứng, robust-float mềm).*

---

## Tóm tắt

Hệ RTK GNSS/IMU/thị giác ghép chặt giải độ mờ số nguyên của pha sóng mang dựa trên hiệp
phương sai float của độ mờ. Các hệ mã nguồn mở SOTA tính hiệp phương sai này bằng một bộ
ước lượng "bóng" chỉ-GNSS vì hiệp phương sai khớp của toàn đồ thị nhân tử là quá đắt để hình
thành trực tuyến — một cách lách chính tác giả chú thích là "hiệp phương sai thô", và nó vứt
bỏ đúng phần thông tin IMU + thị giác mà ghép chặt lẽ ra phải cung cấp. Chúng tôi trình bày
một hiệp phương sai biên độ-mờ **chính xác-trong-cửa-sổ**: nhất quán về mặt thống kê — trùng
với nghiệm `ceres::Covariance` đầy đủ tới cấp tuyến tính hóa — và thời gian thực, qua khử
Schur thưa hai tầng trên cửa sổ hoạt động (khử từng landmark bằng giả-nghịch đảo cắt-hạng,
rồi phân rã đặc cân-bằng-Jacobi có tự-kiểm-chứng bằng tinh chỉnh lặp), **bao gồm cả prior
marginalization**. Trên chuỗi mở-trời 1.787 epoch, phương pháp trùng khớp hiệp phương sai
tham chiếu ở 99,94% epoch (90,1% trong sai số tương đối 1e-3), chỉ bất đồng theo hướng **bảo
thủ** — bất đồng lớn hơn thì bảo thủ (trace lớn hơn) ở 171/177 ca, 6 ca còn lại trace thấp hơn
tham chiếu tối đa 7,3e-4 ngay tại biên 1e-3 (near-tie số học, không phải tự tin quá mức thật),
và chạy 28 ms trung bình so với 812 ms (nhanh ~29×, max 97 ms so với 2,7 s).

Đưa hiệp phương sai nhất quán vào **mọi** epoch làm lộ một vấn đề sâu hơn: ở đô thị sâu, các
lần fix số nguyên *đúng tại chỗ* vẫn làm xấu độ chính xác **phương ngang** vài phút sau đó, vì
mỗi fix được chấp nhận đi vào prior marginalization vĩnh viễn như một ràng buộc gần-cứng
(σ = 0,001 chu kỳ). Qua một chuỗi kiểm sai có đăng ký trước, chúng tôi chỉ ra: (i) kiểm định
tại-thời-điểm-chấp-nhận không thấy được tác hại này (veto thị giác/IMU không bao giờ kích
hoạt — tác hại xuất hiện 4–20 phút sau khi chấp nhận), và (ii) cho ràng buộc fix hết hạn cùng
cửa sổ trượt loại được tác hại nhưng **phá hủy luôn lợi ích** — lợi ích và tác hại nằm cùng
một kênh, đó là sự bền vững của thông tin fix trong prior. Đòn còn lại là *liều lượng*: đánh
trọng số mỗi ràng buộc fix theo độ tin cậy quyết định của chính nó,
thông tin = 1/[(1−P_s) + P_s·1e-6] chu kỳ⁻², với P_s là tỉ lệ thành công integer-bootstrapping
(Teunissen) của tập con được chấp nhận — một moment-match Gaussian của hỗn hợp quyết định,
không có tham số tự do. Qua n=3 lần chạy solo giống-vận-hành trên UrbanNav Deep, cách đánh
trọng-số-theo-tin-cậy cải thiện cao độ ~50% (0,89/0,89/1,23 so với baseline 1,89/2,28 m), yaw,
và độ khả dụng fix ×3,8 — bền vững — nhưng đuôi phương ngang vẫn còn (3,06/3,65/5,23 m). Ta
đi xuống dưới lớp quyết định và robust hóa chính ước lượng float; một loss redescending
triệt-về-không đa số residual GNSS ngay từ epoch đầu và phân kỳ, cho thấy lỗi đô thị sâu là
một *thiên lệch tác động đa số* mà không sơ đồ per-measurement nào cô lập được. Kết luận thực
tế là một phân công rõ: hiệp phương sai nhất quán + đánh trọng số theo tin cậy mang lại lợi
ích cao-độ/yaw/khả-dụng bền vững với kênh ngang giữ ở mức baseline, còn phần đuôi ngang còn
lại nằm ngoài tầm mọi cơ chế trong-bộ-ước-lượng dựa trên thống kê GNSS — nó cần thông tin
cảnh ngoài (phân loại NLOS bằng thị giác, hỗ trợ bản đồ 3D). Mọi can thiệp đều opt-in;
baseline gốc được giữ bất biến từng-byte và kiểm lại sau mỗi lần build. Ground truth chỉ dùng
trong đánh giá hậu kỳ; mọi quy tắc quyết định được đăng ký và đóng băng trước khi chạy, và
mọi lần chạy — kể cả ba can thiệp thất bại — đều được báo cáo.

---

## 1. Giới thiệu

Định vị đô thị chính xác ngày càng dựa vào hợp nhất ghép chặt RTK GNSS, quán tính và thị giác
trong đồ thị nhân tử cửa-sổ-trượt. Sự kiện quyết định độ chính xác trong RTK là giải độ mờ số
nguyên (AR): cố định độ mờ pha về số nguyên biến nghiệm float cấp decimet thành nghiệm fixed
cấp centimet — *khi số nguyên đúng*. Chúng đúng hay không do hiệp phương sai float của độ mờ
đưa vào tìm kiếm số nguyên (LAMBDA) và các phép kiểm chấp nhận quyết định.

Ở đây có một thỏa hiệp cấu trúc trong các hệ mở hiện nay. Trong GICI-LIB [chi23] — nền tảng
GNSS/IMU/camera mở tham chiếu — AR tiêu thụ hiệp phương sai từ một bộ ước lượng "bóng"
chỉ-GNSS song song, thứ mà chính tác giả chú thích là hiệp phương sai thô: hình thành hiệp
phương sai khớp thật của toàn đồ thị trực tuyến bị coi là bất khả thi (chúng tôi đo 687 ms
trung bình mỗi epoch qua `ceres::Covariance` sparse-QR khi cửa sổ đầy — 86% epoch vượt ngân
sách 50 ms). Hiệp phương sai bóng vứt bỏ toàn bộ thông tin IMU và thị giác — chính thông tin
phân biệt một hệ ghép chặt với hệ chỉ-GNSS đúng vào thời điểm quan trọng nhất.

Cách sửa hiển nhiên — để AR dùng hiệp phương sai khớp — đòi hỏi giải đồng thời hai bài toán.
Hiệp phương sai phải **nhất quán** (không tự tin quá mức: chúng tôi chỉ ra một xấp xỉ dựa trên
điều-kiện-hóa rẻ hơn, bỏ phần bù Schur của các trạng thái ngoài-tập, làm phồng độ tự tin và
đẩy bộ ước lượng chấp nhận các fix sai gây trôi, làm xấu RMSE ngang đô thị sâu từ 2,8 m lên
6,8 m), và phải **thời gian thực** ở *mọi* epoch AR, vì hiệp phương sai chỉ thỉnh thoảng khả
dụng sẽ đưa hệ về đường bóng đúng lúc điều kiện khó nhất.

**Đóng góp 1** là hiệp phương sai đó: một biên chính-xác-trong-cửa-sổ Q_aa = [H_active⁻¹]_aa
trên khối độ mờ, tính bằng khử khai thác cấu trúc với cổng tự-kiểm-chứng số học, bao gồm cả
prior marginalization qua một bộ truy cập thông tin chỉ-đọc. Đó là cùng đại lượng
`ceres::Covariance` tính, ở 1/29 chi phí, và không bao giờ tự tin quá mức đáng kể trong 1.787 epoch (thâm hụt trace tệ nhất so với tham chiếu 7,3e-4)
kiểm chứng.

Triển khai hiệp phương sai nhất quán ở mọi epoch trên UrbanNav cho một kết quả bất ngờ, làm
động lực cho phần còn lại của bài: cải thiện bền vững ở cao độ (6/6 cặp, −38..50%), yaw (5/6,
−40..60%) và độ khả dụng fix (×2,4–5) — nhưng độ chính xác **ngang** *xấu hơn* ở 5/6 cặp, đôi
khi tới hàng mét. Chẩn đoán từng-epoch bắt cặp cho thấy các fix chấp nhận là đúng tại chỗ; tác
hại là một đuôi nặng ở các epoch float vài phút sau. Cơ chế là do kiến trúc: mỗi fix chấp
nhận được chèn như ràng buộc gần-cứng (0,001 chu kỳ) mà cửa sổ trượt cuối cùng gộp vào prior
marginalization vĩnh viễn. Một fix hơi lệch — lệch không phải vì số nguyên sai mà vì NLOS đô
thị sâu làm lệch nghiệm float mà nó neo vào — trở thành mỏ neo vĩnh viễn bộ ước lượng không
thể sửa.

**Đóng góp 2** là một chuỗi kiểm sai đăng ký trước biến quan sát này thành tri thức thiết kế.
Với quy tắc quyết định đóng băng và kiểm-cơ-chế không-dùng-GT, chúng tôi thử: cổng chấp nhận
(veto chi phí thị giác/IMU ngưỡng-không cộng cổng bootstrapping P_s ≥ 0,999) — veto không bao
giờ kích hoạt vì tại thời điểm chấp nhận không có gì để thấy; hết-hạn ràng buộc (fix bị xóa
tại marginalization thay vì hấp thụ) — đuôi biến mất nhưng lợi ích cao-độ/yaw cũng biến mất và
fix rate sụp dưới baseline, chứng minh lợi ích và tác hại chung kênh bền-vững; và cuối cùng
**đánh trọng số theo tin cậy quyết định** — mỗi fix tồn tại với thông tin khớp xác suất quyết
định của nó đúng.

**Đóng góp 3** là chính cách đánh trọng số: thông tin(P_s) = 1/[(1−P_s)·1 + P_s·1e-6] chu kỳ⁻²,
là **mô-men bậc hai** (quanh giá trị fix) của hỗn hợp quyết định hai thành phần — đúng với xác
suất P_s ở phương sai ràng buộc gốc (1e-6 chu kỳ²), sai với xác suất 1−P_s lệch ~1 chu kỳ trên
lưới số nguyên (1 chu kỳ²) — dùng làm phương sai ràng buộc; P_s là tỉ lệ thành công
bootstrapping của tập con chấp nhận chính xác (Teunissen; một chặn dưới của tỉ lệ thành công
ILS), tính từ cùng các phương sai điều kiện đã giải tương quan mà LAMBDA dùng. Không hằng số
nào có thể chỉnh tay. Đây là *phồng phương sai* (khớp độ tản của hỗn hợp), không phải *hiệu
chỉnh bias* (nó bỏ qua dấu độ lệch của một fix sai) — một phân biệt hóa ra giải thích chính sự
thất bại của nó, vì tác hại đô thị sâu là một *bias*, không phải phương sai thừa (mục 6c, 8).
Nó là một họ hàng hai-điểm, cấp-ràng-buộc-đồ-thị của ước lượng Best Integer Equivariant — thứ
giữ toàn bộ trọng số hậu nghiệm trên các ứng viên số nguyên.

**Đóng góp 4** mang tính phương pháp luận: các phát hiện về giao thức cho benchmark GNSS dựa
trên replay (replay post-file của nền tảng tham chiếu bị điều-áp-ngược khiến kết quả nhạy tải;
bắt-cặp cùng-tải hoặc chạy solo-máy-rảnh là bắt buộc), một sàn nhiễu giao thức đã hiệu chuẩn
(~0,2 m, từ các chuỗi mà các thuật toán so sánh chứng minh được là giống hệt), và một kỷ luật
thực nghiệm đăng-ký-trước hoàn chỉnh (bins quyết định đóng băng, kiểm cơ chế, addendum có
ngày) mà chúng tôi lập luận nên là chuẩn cho nghiên cứu AR, nơi cám dỗ chỉnh ngưỡng chấp nhận
theo GT là lớn và vô hình trong các con số công bố.

## 2. Công trình liên quan

*[Hoàn tất sau vòng khảo cứu sâu — các mỏ neo chính:]*
- **GICI-LIB** [chi23] và cách lách hiệp phương sai bóng; GVINS, IC-GVINS, VINS-Fusion làm
  baseline hợp nhất (đối chiếu Bảng V của [chi23]).
- **Lý thuyết ước lượng số nguyên**: LAMBDA và MLAMBDA; ratio test và biến thể tỉ-lệ-thất-bại
  cố định (FFRT); giải độ mờ từng phần; **tỉ lệ thành công bootstrapping** P_s [teunissen98]
  làm thước chất lượng quyết định tiên nghiệm. Gần nhất về tinh thần là **ước lượng Best
  Integer Equivariant (BIE)** [teunissen03, odolinski20], vốn đánh trọng số các ứng viên số
  nguyên theo xác suất hậu nghiệm thay vì chốt một; trọng số ràng buộc của ta là một hỗn hợp
  BIE hai-điểm áp ở cấp ràng-buộc-đồ-thị, giữ nguyên pipeline LAMBDA chuẩn.
- **Tính nhất quán trong bộ ước lượng cửa sổ trượt**: prior marginalization, FEJ; bệnh lý
  tự-tin-quá-mức.
- **GNSS đô thị sâu**: giảm thiểu NLOS/đa đường, 3DMA, mô hình lỗi robust — bối cảnh cho giới
  hạn chúng tôi thiết lập.

## 3. Tổng quan hệ

Chúng tôi xây trên bộ ước lượng ghép chặt RTK/IMU/camera của GICI-LIB ("RRR"): đồ thị nhân tử
cửa-sổ-trượt trên nền Ceres với nhân tử pseudorange/phaserange sai-phân-kép GNSS, tiền-tích-phân
IMU, nhân tử tái chiếu, và prior marginalization đặc; AR tạo sai phân đơn giữa-vệ-tinh, giải
các lane UWL/WL/NL bằng MLAMBDA với fix từng phần sắp theo elevation/phương-sai/phân-số, áp số
nguyên chấp nhận như nhân tử ràng buộc (σ = 0,001 chu kỳ) cộng một cập nhật Kalman nội bộ, và
kiểm bằng phép kiểm chi-phí-range ngưỡng-không. Bộ ước lượng của chúng tôi
(`rtk_imu_camera_rrr_va`) là một biến thể đăng ký; bộ ước lượng gốc và mặc định của nó không
đụng tới (dung sai baseline đóng băng được kiểm lại sau mỗi build: APE 0,0290–0,0292 m /
0,470–0,485° qua bốn lần build lại hôm nay so với mốc khóa 0,029198 m / 0,470557° ± 0,005/0,1).

## 4. Hiệp phương sai độ mờ biên nhất quán, thời gian thực

**Bài toán.** AR cần Q_aa = [H⁻¹]_aa với H là thông tin Gauss-Newton của *toàn cửa sổ hoạt
động* — GNSS, IMU, thị giác và prior marginalization — đánh giá tại ước lượng hiện tại, `aa`
đánh chỉ số các khối độ mờ. `ceres::Covariance` tính đúng cái này nhưng ở chi phí O(sparse-QR
toàn bài); một xấp xỉ điều-kiện-hóa cục bộ (bỏ H_an H_nn⁻¹ H_na cho các khối ngoài-tập) nhanh
nhưng tự tin quá mức — kết quả âm đã ghi ở trên.

**Phương pháp.** Ta lắp H = Σ_r J_rᵀ J_r trên mọi residual hoạt động trong tọa độ tối thiểu,
với hai điểm quyết định tính đúng đắn: (1) residual đã robust hóa đóng góp Jacobian
**đã-hiệu-chỉnh-Triggs** (khớp corrector của Ceres), không phải thô; (2) prior marginalization
đóng góp qua một bộ truy cập chỉ-đọc mới phơi ra Lambda = JᵀJ nội bộ kèm offset tối thiểu
từng-khối — không bao giờ qua đường evaluate tổng quát (mà buffer động của nó là đường dễ
crash). Rồi ta khử hai tầng khai thác cấu trúc đồ thị:

- *Tầng 1 — landmark.* Mỗi khối landmark 3×3 H_ll chỉ nối với pose. Ta khử từng landmark bằng
  phần bù Schur giả-nghịch-đảo cắt-hạng; lập luận nửa-xác-định-dương là các mode null của một
  khối landmark có hàng nối bằng không, nên cắt là *chính xác*, không phải xấp xỉ.
- *Tầng 2 — nuisance đặc.* Khối nuisance còn lại (pose, vận tốc/bias, đồng hồ, tần số, ngoại
  lai, biến chỉ-prior) được cân bằng Jacobi (bất đồng nhất đơn vị, không phải suy biến, chi
  phối điều kiện thô của nó) và phân rã bằng LDLT đặc với ba bước tinh chỉnh lặp. Residual
  tinh chỉnh đóng vai trò **tự-kiểm-chứng số học**: kết quả chỉ được nhận nếu hiệu ứng phần bù
  Schur của số gia cuối dưới thang trị riêng nhỏ nhất của hệ đã khử — cổng lỗi-số-học không
  hằng-số-chỉnh-tay. Khi thất bại (lý do bỏ-cuộc có kiểu được ghi log), lời gọi rơi về hiệp
  phương sai bóng; ta không bao giờ ngụy tạo độ tin cậy.

**Kiểm chứng (Bảng 1).** Toàn quỹ đạo GICI-board 1.1, 1.787 epoch AR, so sánh từng-epoch với
`ceres::Covariance`:

| chỉ số | giá trị |
|---|---|
| epoch dùng được (đường nhanh) | 99,94% |
| sai số tương đối ≤ 1e-3 | 90,1% |
| bất đồng > 1e-3 | 177 — bảo thủ (trace lớn hơn) 171; trace dưới tham chiếu 6, thâm hụt tệ nhất **7,3e-4** tại biên 1e-3 (near-tie số học) |
| thời gian trung bình / max (nhanh) | 28 ms / 97,5 ms |
| thời gian trung bình / max (ceres) | 812 ms / 2.714 ms |
| an toàn heap | ASan sạch, 1.761 epoch, 0 lỗi |

Hồ sơ bất đồng bảo-thủ-hoặc-hòa là tính chất an-toàn-AR: chế độ hỏng của chuỗi rơi-về số học
làm *phồng* độ bất định, không bao giờ làm xẹp đáng kể (6 ca dưới tham chiếu là near-tie
0,02–0,07% trace tại ngưỡng 1e-3, không phải tự tin quá mức). Thước ở đây là *trace* hiệp
phương sai (đại lượng log online); kiểm per-ambiguity-đường-chéo (đúng đại lượng cổng AR tiêu
thụ) để dành cho phụ lục phản biện. Độ chính xác và số fix mở-trời được giữ
(0,028991 m / 0,4727°, 869/1775 fixed so với baseline 876/1775).

## 5. Đánh giá UrbanNav: câu đố phương ngang

**Giao thức.** UrbanNav-HK Medium (TST) và Deep (Whampoa); RMSE ENU thô so với GT nội suy về
mốc thời gian nghiệm, **không căn chỉnh** (thước liên quan vận hành; §8 bắc cầu sang ATE/APE
căn chỉnh); cổng đầy đủ n_matched ≥ 14.000/15.119; lặp n=3; và — sau khi phát hiện replay
post-file của nền tảng bị điều-áp-ngược (không có định-nhịp đồng-hồ trong vòng đọc; cùng chuỗi
25,2 phút chạy 14–24 phút wall-clock với cùng khối lượng) trong khi ngân sách solver là
wall-clock — **bắt cặp cùng-tải**: mỗi cặp so sánh chạy cả hai thuật toán đồng thời; không bao
giờ so qua các chế độ tải khác nhau. Medium — nơi cả hai thuật toán chấp nhận 0 fix nên *giống
hệt về logic* — hiệu chuẩn sàn nhiễu giao thức ~0,2 m; ta gắn cờ mọi delta nhỏ hơn là nhiễu,
kể cả "cải thiện" Medium của chính mình.

**Kết quả (Deep, bắt cặp, n=3 mỗi bên; Bảng 3).**

| arm | rmse_h (m) | rmse_u (m) | yaw (độ) | fixed |
|---|---|---|---|---|
| baseline (matrix-1) | 3,026 ± 0,679 | 2,266 ± 0,267 | 1,089 ± 0,279 | 1,7% |
| VA-v1 (hiệp phương sai nhất quán) | 3,401 ± 0,784 | 1,463 ± 0,213 | 0,657 ± 0,073 | 8,8% |
| baseline (mới, đợt v2) | 3,031 ± 0,439 | 2,871 ± 0,613 | 1,743 ± 0,903 | 1,3% |
| VA-v2 (+ cổng) | 4,614 ± 2,141 | 1,780 ± 0,211 | 0,783 ± 0,045 | 4,1% |

Qua cả sáu cặp hợp lệ: cao độ tốt hơn 6/6 (sign test p = 0,016), yaw 5/6 — và ngang **xấu hơn
5/6** (trung vị +0,62 m, tệ nhất +3,65 m). Baseline ngang tái lập gần như y hệt qua các giao
thức (3,026 so với 3,031), nên thiết kế vững; hiệu ứng là thật.

**Chẩn đoán (Bảng 4).** Điều kiện hóa delta lỗi ngang từng-epoch bắt cặp theo trạng thái fix
lật ngược lời giải thích ngây thơ: tại các epoch VA-fixed, arm VA tốt-hơn-hoặc-bằng baseline
bắt cặp (trung vị dh −0,03/−0,01/−1,01 m mỗi đợt); lỗi thừa nằm ở đuôi nặng tại các epoch
*float* (đợt vỡ: p50 2,77 so với 1,62 m nhưng p99 27,96 so với 9,59 m), trong các đoạn thảm
họa xảy ra 4–20 phút *sau lần fix chấp nhận cuối* (mọi fix trước t = 267 s; các đoạn vỡ ở
t = 528–630 s và 1492–1512 s, đoạn sau 47 m so với baseline 1,0 m cùng thời điểm). Kênh nhân
quả duy nhất sống sót trước mốc-thời-gian này là prior marginalization: fix chèn ở σ = 0,001
chu kỳ và cuối cùng bị marginalize vào một prior không bao giờ quên; một fix neo vào nghiệm
float lệch-NLOS trở thành nguồn thiên lệch gần-cứng, vĩnh viễn. Đáng chú ý, hiểm họa này vô
hình với mọi thước chất lượng tại-thời-điểm-chấp-nhận — các tập con chấp nhận mang tỉ lệ thành
công bootstrapping P_s ≥ 0,999995 dưới mô hình Gaussian, mà giả định kỳ-vọng-không chính là
thứ NLOS vi phạm.

## 6. Chuỗi kiểm sai

Mọi can thiệp dưới đây đều đăng ký trước với bins quyết định đóng băng (tail event := đợt/run
bất kỳ có d_h > +1,0 m; SUCCESS := không tail AND trung vị d_h ≤ +0,2 m AND giữ cao-độ/yaw ở
≥ 2/3) và kiểm-cơ-chế không-dùng-GT; mọi sai lệch mang addendum có ngày.

**(a) Cổng chấp nhận (VA-v2).** Một veto chi phí thị giác/IMU (từ chối nếu chi phí tái chiếu+IMU
đã robust hóa tăng sau lần giải-lại có ràng buộc — bản sao đúng của quy tắc chi-phí-range của
nền tảng, không hằng số mới) và cổng tập con P_s ≥ 0,999. Kết quả: veto kích hoạt **0 lần ở
mọi run trên mọi bộ dữ liệu** — nhất quán với chẩn đoán: tại thời điểm chấp nhận không có gì
để thấy. Cổng P_s giảm nửa fix rate (8,8% → 4,1%) giữ lợi cao-độ/yaw, nhưng đuôi ngang vẫn còn
(đợt tệ nhất +3,65 m). *Cổng lọc quyết định; tác hại không phải thuộc tính của quyết định.*

**(b) Hết hạn ràng buộc (VA-v3).** Ràng buộc fix bị xóa tại marginalization — quyết định giữ
cứng trong cửa sổ sống nhưng không bao giờ vào prior. Cơ chế đã kiểm (21.308 ràng buộc bị xóa;
hai lỗi bất-biến marginalizer được ghi cho người tái lập: khối hết-residual vi phạm phép kiểm
kết-nối, và khối được prior tạm-gỡ tham chiếu phải marginalize *qua* nó, không được xóa). Kết
quả: hết vỡ thảm họa, nhưng fix rate sụp dưới baseline (0,1–1,3%), lợi cao độ **đảo chiều**
(3,53 so với 1,82 m), và một tail event còn lại (+1,24 m) — **THẤT BẠI**, và kiểu thất bại giàu
thông tin: *lợi ích và tác hại sống cùng một kênh — sự bền vững của thông tin fix trong prior.
Một công tắc nhị phân giữ/bỏ không tách được chúng.*

**(c) Đánh trọng số theo tin cậy (VA-v4).** Liều lượng: thông tin(P_s) như §1, P_s của tập con
chấp nhận chính xác (đường LAMBDA: lưu tại chấp nhận; đường làm-tròn: tính trên khối trên-trái
hiệp phương sai chấp nhận). Fix tin-cậy-cao giữ gần-cứng (P_s → 1 khôi phục ràng buộc gốc);
fix sát mép tồn tại với trọng số bị chặn trung thực (P_s = 0,999 → σ = 0,032 chu kỳ, giảm
1000× thông tin). Kiểm cơ chế: trên Deep, 44/445 tập con chấp nhận mang ràng buộc mềm đáng kể
(tới σ = 0,62 chu kỳ); trên mở-trời, P_s ~ 1 xuyên suốt và hành xử y hệt gốc (số fix 818/1775;
độ chính xác trong dải đóng băng) — liều lượng chỉ kích hoạt đúng nơi mô hình kém chắc chắn.
Kết quả, n=3 solo Deep: cao độ cực-ổn-định và ~50% tốt hơn baseline (u = 0,894/0,887/1,234 m
so với BL 1,893/2,282), yaw tốt hơn, fix ×3,8 — nhưng đuôi ngang vẫn còn (h = 3,06/3,65/**5,23**
m; run3 là tail event) — **THẤT BẠI của H2**. Hai lý do cộng dồn, đều do cấu trúc: (i) tác hại
vô hình với mọi thước tin-cậy tại-chấp-nhận — các tập con chấp nhận mang P_s ≥ 0,999995 dưới
mô hình Gaussian mà giả định kỳ-vọng-không bị NLOS vi phạm, nên liều gần-cứng đúng nơi một fix
sai gây hại nhất; (ii) ngay cả nơi liều có làm mềm, nó phồng *phương sai* ràng buộc, trong khi
tác hại là một *bias* (fix neo vào float lệch có hệ thống) — một điều chỉnh mô-men-bậc-hai
không thể triệt một lỗi mô-men-bậc-một. *Không lớp quyết định dẫn-xuất-từ-hiệp-phương-sai nào
— cổng hay liều — có thể thấy, do đó không thể triệt, một thiên lệch mà chính hiệp phương sai
không biểu diễn.* Thang quyết định đóng lại.

**(d) Robust hóa float (RF).** Đòn trong-bộ-ước-lượng cuối đi xuống dưới lớp quyết định, tới
chính ước lượng float: thay loss Huber ảnh-hưởng-không-chặn của gốc trên mọi residual GNSS
bằng Tukey biweight redescending (c = 4,685, hằng 95%-hiệu-suất; không tham số tự do), để đo
lệch-NLOS mất ảnh hưởng *trước khi* AR chạy. Kết quả: bộ ước lượng **phân kỳ và crash** lúc
t ~ 160 s. Log cơ chế dứt khoát và kích hoạt từ epoch đầu — Tukey triệt-về-không *đa số hoặc
toàn bộ* residual GNSS ở 40/48 epoch ghi log (25 lần triệt 5/5) — vì ở đô thị sâu dải residual
tự nhiên tại điểm tuyến tính hóa đã vượt 4,685 sigma; kernel redescending do đó bỏ đói float
mất mỏ neo GNSS và mất basin thu hút. Một biến thể mềm (Cauchy, c = 2,3849, giữ gradient khắp
nơi) chạy trọn không crash và nhẹ (giảm 2–3 của ~70 residual mỗi epoch, không triệt khối):
lần thử đầu không đuôi (h = 2,73 m) — đủ hứa hẹn để chúng tôi đăng ký trước và chạy một tập
n=3 xác nhận (run 2–3 mù, bins đóng băng, run1 công khai). Kết quả xác nhận là **THẤT BẠI**:
h = 2,73 / **4,25** / 3,56 m (trung bình 3,51 ± 0,76) — run2 là tail event. run1 không đuôi là
một draw may, đúng như trạng thái n=1 và biên 0,14 m dưới-sàn-nhiễu đã cảnh báo. Cauchy chỉ
giảm nhẹ 2–3/70 residual — quá nhẹ trước thiên lệch tác động đa số — nên giữ lợi cao-độ/yaw
của VA-v4 (u tốt hơn baseline ở 2/3, yaw ~0,7°) nhưng không thêm gì cho ngang. *Robust hóa
per-residual không thể cô lập các đo xấu vì quá nhiều đo bị lệch cùng lúc: lỗi đô thị sâu là
hiện tượng tác-động-đa-số mà không cơ chế chỉ-thống-kê-GNSS nào giải được.* Điều này minh oan
cho kỷ luật đăng-ký-trước: một tuyên bố từ run đơn run1 sẽ sai.

## 7. Kết quả cuối

Giao thức solo giống-vận-hành (mỗi run một mình trên máy rảnh; baseline n=2 sau khi mất một run
lúc teardown phiên, VA-v4 n=3; bins đăng ký trước). Deep, ENU thô (thước chính):

| arm (solo, Deep) | rmse_h (m) | rmse_u (m) | yaw (độ) | fixed |
|---|---|---|---|---|
| baseline (n=2) | 3,230 / 2,508 | 1,893 / 2,282 | 0,802 / 0,843 | 1,6% / 0,9% |
| **VA-v4 (liều, n=3)** | 3,06 / 3,65 / 5,23 (μ 3,98) | **0,894 / 0,887 / 1,234 (μ 1,01)** | 0,642 / 0,765 / 0,685 | 6,1 / 5,9 / 7,0 % |
| RF-Cauchy (float mềm, n=3) | 2,73 / 4,25 / 3,56 (μ 3,51) | 1,001 / 1,579 / 2,115 (μ 1,57) | 0,874 / 0,670 / 0,728 | 9,7 / 11,1 / 7,1 % |
| VA-v2 (cổng) — tham chiếu | 3,534 | 2,328 | 0,952 | 3,5% |
| VA-v3 (hết hạn) — tham chiếu | 3,071 | 4,063 | 1,134 | 0,3% |
| RF-Tukey (float cứng) — tham chiếu | crash @160 s | — | — | — |

Cả VA-v4 lẫn RF-Cauchy đều mang đuôi ngang (VA-v4 run3 5,23 m; RF-Cauchy run2 4,25 m) trong khi
cải thiện cao độ (μ ~1,0–1,6 m so với baseline ~2,1 m), yaw và fix rate. Câu chuyện nhất quán
qua các giao thức bắt-cặp và solo, năm can thiệp, và cả hai dạng robust-loss: **cao độ và yaw
cải thiện bền vững và đáng kể; khả dụng fix nhân lên; ngang mang đuôi nặng không cơ chế
trong-bộ-ước-lượng nào trên thống kê GNSS loại được.** VA-v4 là cấu hình khuyến nghị khi vận
hành coi trọng cao độ, yaw và khả dụng fix và có thể giữ kênh ngang ở mức float-baseline.

**T7 — bắc cầu ba tầng thước đo (Deep solo).** ENU thô (chính, vận hành), ATE SE(3) (căn
xoay+tịnh-tiến, không co giãn — chuẩn hệ metric), APE Sim(3) (căn xoay+tịnh-tiến+co-giãn —
thước của Bảng V [chi23]; mốc deep RRR 2,46 m / 1,64°). Căn chỉnh dần giấu đi thiên lệch/co
giãn toàn cục, nên các tầng đọc từ khắt-khe→dễ-dãi.

| run | ENU thô h | ATE SE(3) | APE Sim(3) m/độ |
|---|---|---|---|
| baseline r1 | 3,230 | 2,495 | 2,494 / 0,978 |
| baseline r2 | 2,508 | 2,234 | 2,183 / 1,014 |
| VA-v4 r1 | 3,06 | 2,358 | 2,341 / 0,779 |
| VA-v4 r2 | 3,65 | 2,252 | 2,097 / 0,869 |
| VA-v4 r3 (đuôi) | 5,23 | 3,567 | 3,422 / 0,833 |
| RF-Cauchy r1 | 2,73 | 1,972 | 1,886 / 1,009 |
| RF-Cauchy r2 (đuôi) | 4,25 | 2,978 | 2,870 / 0,859 |
| RF-Cauchy r3 | 3,56 | 2,696 | 2,655 / 0,955 |

Hai điều ghi trung thực: (i) ngay dưới Sim(3), baseline của ta (2,18–2,49) *ôm trọn* mốc
run-đơn 2,46 của [chi23], khẳng định baseline tái lập đúng chuẩn và so sánh công bằng; (ii)
lợi cao-độ/yaw sống sót qua căn chỉnh trong khi đuôi ngang cũng sống sót (VA-v4 run3 3,42;
RF-Cauchy run2 2,87 Sim(3)) — căn chỉnh không tạo ra lợi cao-độ/yaw cũng không giấu được đuôi.

## 7a. Cấu hình khuyến nghị và hợp đồng triển khai

Chúng tôi phát biểu kết quả như một hợp đồng tường minh thay vì một con số headline, vì giá
trị của phương pháp phụ thuộc kênh và người triển khai cần biết nó dịch chuyển kênh nào.

**Cấu hình khuyến nghị.** Hiệp phương sai biên nhất quán thời gian thực
(`ar_use_fast_marginal_covariance`) + đánh trọng số fix theo tin cậy quyết định
(`use_success_rate_fix_information`), tức VA-v4. Thời gian thực (hiệp phương sai mỗi epoch AR
28 ms trung bình); opt-in; baseline gốc bất biến từng-byte khi tắt.

**Hợp đồng (UrbanNav Deep, RTK/IMU/thị giác ghép chặt):**

| Kênh | Hiệu ứng so với float baseline | Trạng thái |
|---|---|---|
| Cao độ (up) | ~50% tốt hơn, phương sai thấp (0,9–1,2 m so với 1,9–2,3 m) | **cải thiện — tuyên bố** |
| Yaw | tốt hơn (0,64–0,77° so với 0,80–0,84°) | **cải thiện — tuyên bố** |
| Khả dụng fix | ×3,8 (6–7% so với 1,6%) | **cải thiện — tuyên bố** |
| Tính nhất quán hiệp phương sai | bảo-thủ-hoặc-hòa (171/177 lớn hơn, 6 trong 7,3e-4 trace) | **đã chứng minh** |
| Thời gian thực | 28 ms trung bình / epoch (so với 812 ms chính xác) | **đã chứng minh** |
| Ngang (E/N) | giữ ở mức baseline; đuôi nặng còn, không tuyên bố cải thiện | **không cải thiện** |
| Ngang qua bất kỳ đòn in-estimator nào trên thống kê GNSS | cổng, hết-hạn, liều theo tin cậy, và M-estimation float đều thất bại | **chứng minh ngoài tầm** |

**Ý nghĩa cho người triển khai.** Dùng VA-v4 khi độ chính xác cao độ (ví dụ phân biệt đường
nhiều tầng), heading và khả dụng fix là yêu cầu ràng buộc và lỗi ngang chấp nhận được ở mức
float-baseline. KHÔNG kỳ vọng cải thiện độ chính xác ngang ở đô thị sâu từ phương pháp này hay
bất kỳ cơ chế nào dựa trên thống kê GNSS; điều đó cần thông tin cảnh ngoài.

**Hướng tương lai (một dòng, không mở thang in-estimator mới).** Đuôi ngang là hiện tượng
thiên-lệch-NLOS-đa-số; giải nó đòi xác định *vệ tinh nào* không-trực-xạ từ cấu trúc cảnh —
phân loại LOS/NLOS bằng thị giác hoặc hỗ trợ bản đồ 3D — áp lên ước lượng float trước AR, giữ
VA-v4 phía trên. Đây là một hệ riêng và một nghiên cứu riêng.

## 8. Thảo luận

**Điều đã thiết lập.** (i) Một hiệp phương sai độ mờ khớp nhất quán tính được thời gian thực;
tính chất an toàn của nó (bảo-thủ-hoặc-hòa — không bao giờ tự tin quá mức đáng kể, thâm hụt
trace tệ nhất 7,3e-4) đúng trên mọi epoch kiểm chứng. (ii)
Với hiệp phương sai nhất quán, thông tin thị giác/IMU cải thiện bền vững kênh cao độ, yaw và
khả dụng fix ở đô thị sâu. (iii) Kênh ngang bị chi phối không phải bởi chất lượng quyết định
mà bởi *sự bền vững* quyết định: các fix đúng tại chỗ, neo vào trạng thái float lệch-NLOS, trở
thành mỏ neo prior vĩnh viễn. Cổng không thấy được (veto không kích hoạt); hết-hạn vứt bỏ lợi
ích cùng tác hại; đánh trọng số theo tin cậy giữ lợi ích nhưng không giữ đuôi; và robust hóa
chính float thì hoặc phân kỳ (Tukey redescending triệt khối lệch-đa-số) hoặc, làm mềm thành
Cauchy, quá nhẹ để dịch một thiên lệch tác-động-đa-số. Năm can thiệp đăng-ký-trước hội tụ về
một kết luận: **đuôi ngang đô thị sâu nằm ngoài tầm mọi cơ chế chỉ dựa trên thống kê GNSS.**

**Giới hạn.** Mọi đại lượng tin-cậy và trọng-số trong pipeline (P_s, ratio, cổng phương sai,
trọng số robust) dẫn xuất từ hiệp phương sai float Gaussian, mà giả định kỳ-vọng-không bị NLOS
vi phạm; ở đoạn dễ các thước này bão hòa ở "chắc chắn" trong khi chế độ hỏng thật vô hình, và
ở đoạn khó quá nhiều đo bị lệch cùng lúc để bất kỳ sơ đồ per-measurement nào cô lập các đo xấu.
Nguồn thông tin kế tiếp phải là ngoài và phải xác định *vệ tinh nào* bị chắn từ cấu trúc cảnh:
phân loại LOS/NLOS bằng thị giác (nền tảng đã có camera) hoặc hỗ trợ bản đồ 3D — chủ đề của
nghiên cứu đồng hành. Ta cũng gắn cờ trung thực: kết quả Medium là ngang-bằng-do-cấu-tạo (0
fix cả hai bên — delta ở đó đo sàn nhiễu giao thức); baseline là n=2 trên giao thức solo sau
khi mất một run lúc teardown (phán quyết vững trước điều đó); kết quả Deep mang thống kê n=3
dưới một giao thức replay có nhạy-tải mà ta đặc trưng hóa nhưng không loại bỏ được.

**Liêm chính.** GT chỉ xuất hiện trong đánh giá hậu kỳ; mọi quy tắc quyết định được đóng băng
trước khi chạy; mọi sai lệch là addendum có ngày; mọi lần chạy được báo cáo, kể cả ba thiết kế
can thiệp thất bại và hai lần crash cài đặt. Chúng tôi tin kỷ luật này thay đổi thực chất độ
tin cậy của nghiên cứu AR, nơi ngưỡng chấp nhận theo lệ thường được chỉnh trên chính bộ dữ
liệu đánh giá.

## 9. Kết luận

Nhất-quán-và-thời-gian-thực là khả thi cho hiệp phương sai độ mờ khớp, và nó dịch chuyển kim
đồng hồ ở nơi mô hình thấy được (cao độ, yaw, khả dụng). Ở nơi mô hình không thấy (ngang
lệch-NLOS), thất bại là do kiến trúc, và một chuỗi năm-bước đăng-ký-trước — cổng, hết-hạn,
liều, float robust cứng, float robust mềm — thiết lập rằng không cơ chế nào trên thống kê GNSS
loại được nó, cô lập cách đánh trọng số theo tin cậy quyết định (VA-v4) làm cấu hình giữ được
lợi cao-độ/yaw/khả-dụng trong khi giữ ngang ở baseline. Vượt qua ranh giới ngang cần thông tin
cảnh ngoài, mà chúng tôi bàn ở nghiên cứu đồng hành. Hiệp phương sai nhất quán, sơ đồ trọng
số, và giao thức kiểm sai đều tương thích-với-gốc và opt-in.

---
*Khả tái lập: mọi config, đăng-ký-trước kèm addendum có ngày, log chạy, và script đánh giá đều
ở trong repo; baseline gốc bất biến từng-byte về hiệu quả và được kiểm lại mỗi build.*
