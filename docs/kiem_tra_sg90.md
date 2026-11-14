# Kiểm tra servo SG90 của kẹp (khi bước B3 không làm kẹp chuyển động)

Tài liệu này dùng khi firmware `calib` trả lời đúng (`OK ON G`, `AT G <µs>`) nhưng kẹp đứng yên. Mục tiêu là tìm ra **khâu nào** trong chuỗi phát xung → dây → servo → bánh răng → ngón kẹp đang hỏng, bằng các phép thử rẻ và không phá hủy trước.

## 1. Điều đã biết và suy ra được

- Firmware nhận lệnh và báo `AT G`, nên Serial và vòng lặp firmware vẫn chạy.
- J1, J2 và J3 chạy đúng với **cùng** firmware `calib` và cùng lớp `ServoPwm`. Kênh G chỉ khác ở chân (GPIO33) và số kênh LEDC (3). Vì vậy lỗi phần mềm ít có khả năng (suy luận từ `src/calib/main.cpp`).
- SG90 lấy nguồn từ Buck 2 (5 V), không phải từ Buck 1 như ba MG996R (`mk2_connection_diagram.html`, Hình 1). Bước B0.1 lại yêu cầu tháo dây OUT+ Buck 2 khỏi chân 5V của ESP32. Nếu chỗ tháo nằm ở mối hàn chung thì SG90 cũng mất nguồn.
- Bước A1 yêu cầu tháo bánh răng kẹp. Nếu bánh răng chưa ăn khớp lại hoặc bị trờn răng thì trục servo có thể quay mà ngón kẹp không chuyển động.

## 2. Giả thuyết

| # | Giả thuyết | Mức tin (ước lượng của người soạn) | Dấu hiệu |
|---|---|---|---|
| H1 | SG90 không có nguồn 5 V (dây dương bị tháo theo B0.1, hở mối hàn, hoặc Buck 2 không ra áp) | Trung bình–cao | Servo im lặng hoàn toàn, trục cầm nhẹ xoay được |
| H2 | Dây tín hiệu S không tới GPIO33 (chưa cắm lại sau B0.3, cắm nhầm IO32/IO35 nằm sát IO33, đứt dây) | Trung bình | Như H1, nhưng đo được 5 V ở chân dương của servo |
| H3 | Lỗi cơ khí: bánh răng kẹp chưa ăn khớp, trờn răng, hoặc ngón kẹp kẹt | Trung bình | Trục hoặc bánh răng servo quay nhưng ngón không đi, hoặc servo rè mà không quay |
| H4 | SG90 hỏng | Thấp–trung bình | Có 5 V, có xung đúng tới tận đầu dây, nhưng servo không phản ứng khi đã tháo khỏi kẹp |
| H5 | GPIO33 hoặc kênh LEDC không phát xung | Thấp | Không đo được xung ở chân IO33 trên header |

## 3. Chương trình `servo_test`

```bash
pio run -e servo_test -t upload
pio device monitor -b 115200        # phai thay "# READY servo_test"
```

| Lệnh | Tác dụng |
|---|---|
| `US <µs>` | Phát xung `<µs>` (kẹp trong 900–2100) trên chân đang chọn |
| `SWEEP [lo hi]` | Quét qua lại `lo`..`hi` (mặc định 1300..1700), tốc độ `RATE` |
| `RATE <µs/s>` | Tốc độ quét, 10–500 (mặc định 200) |
| `OFF` | Ngừng xung |
| `PIN <25\|26\|27\|33>` | Đổi chân phát xung (mặc định 33). Dùng để đổi chéo servo và chân |
| `MEAS` | Đo độ rộng xung trên **GPIO34**. Nối một dây từ điểm cần kiểm tra vào chân **IO34** (header J2, chân 5) |
| `?` | Trạng thái |

GPIO34 là chân chỉ đọc vào và không dùng ở nơi nào khác trong robot. `MEAS` dùng `pulseIn()` nên sai số cỡ vài µs. Như vậy là đủ để phân biệt "có xung đúng" với "không có xung".

## 4. Quy trình thử (làm theo thứ tự)

An toàn nguồn giống bước B0: dây OUT+ Buck 2 **không** nối vào chân 5V của ESP32 khi đang cắm USB, mọi dây GND giữ nguyên. Trước mỗi lần cắm hoặc tháo dây, tắt S1.

### T0. Quan sát (không cần dụng cụ)
Nạp `servo_test`, bật S1, gõ `SWEEP`. Nhìn trục ra và bánh răng của SG90, áp tai nghe.

| Kết quả | Suy ra |
|---|---|
| Trục servo quay qua lại nhưng ngón kẹp đứng yên | **H3**: kiểm tra bánh răng kẹp có ăn khớp không, có trờn răng không. Dừng ở đây |
| Servo rè hoặc rung nhưng không quay | **H3** (kẹt cơ khí) hoặc thiếu áp. Gõ `OFF`, làm T1 |
| Hoàn toàn im lặng | Làm T1 |

### T1. Đo nguồn tại đầu dây servo
Đồng hồ vạn năng thang DC V, que đen vào dây âm (nâu), que đỏ vào dây dương (đỏ) của SG90, càng gần thân servo càng tốt (màu dây SG90 theo nguồn [7] của sơ đồ đấu nối).

| Kết quả | Suy ra |
|---|---|
| ≈ 0 V | **H1**. Kiểm tra mối hàn dây dương SG90 với OUT+ Buck 2. Nếu B0.1 đã cắt luôn nhánh này thì nối lại nhánh SG90 nhưng **vẫn để hở** nhánh tới chân 5V ESP32 |
| ≈ 5,0 V | Nguồn ổn, làm T2 |
| Thấp hơn rõ rệt (ví dụ < 4,5 V) | Buck 2 đang giới hạn dòng hoặc chỉnh sai. Kiểm tra đèn CC và áp không tải của Buck 2 |

### T2. Kiểm tra xung tại chân IO33 trên header
Tháo dây S của SG90 khỏi IO33. Nối một dây jumper từ **IO33** sang **IO34**. Gõ `US 1500` rồi `MEAS`, sau đó `US 1800` rồi `MEAS`.

| Kết quả | Suy ra |
|---|---|
| Đo được ≈ 1500 và ≈ 1800 µs | GPIO33 phát xung đúng, loại H5. Làm T3 |
| `KHONG CO XUNG` | Gõ `PIN 25`, chuyển jumper sang IO25 → IO34 và đo lại. Nếu IO25 có xung còn IO33 không có thì **H5** (chân IO33 hoặc mối hàn header). Nếu cả hai đều không có thì kiểm tra lại chính dây jumper |

### T3. Kiểm tra xung tại đầu dây servo
Cắm lại dây S của SG90 vào IO33. Rút đầu connector của SG90 ra khỏi servo (nếu có nối dây trung gian) hoặc chọc một dây nhỏ vào lỗ chân S trên connector. Nối điểm này về IO34 và `MEAS`.

| Kết quả | Suy ra |
|---|---|
| Đo được xung đúng | Xung tới tận servo, loại H2. Làm T4 |
| `KHONG CO XUNG` | **H2**: dây hoặc connector tín hiệu bị đứt, hoặc dây đang cắm vào IO32/IO35 thay vì IO33. Dò lại theo nhãn in trên board |

### T4. Đổi chéo với một kênh đã chạy tốt
Tắt S1. Cắm dây S của SG90 vào IO25 (chỗ của J1) và tháo dây S của J1 ra. Bật S1, gõ `PIN 25` rồi `SWEEP`.

| Kết quả | Suy ra |
|---|---|
| SG90 chạy | Servo tốt. Lỗi nằm ở đường IO33 hoặc dây của nó (quay lại T2, T3) |
| SG90 vẫn đứng yên (đã có 5 V ở T1) | Làm T5 |

### T5. Thử servo trần
Tháo SG90 khỏi cơ cấu kẹp hoặc gỡ bánh răng để trục quay tự do. Giữ nguồn 5 V từ Buck 2 và dây S ở chân đã chứng minh có xung (T2 hoặc T4). Gõ `SWEEP 1000 2000`.

| Kết quả | Suy ra |
|---|---|
| Servo trần quay | Servo tốt, lỗi là **H3**: cơ cấu kẹp kẹt hoặc lệnh đang đẩy ngón vào điểm chặn. Kiểm tra ngón trượt tự do bằng tay khi servo tắt |
| Servo trần vẫn không quay dù có 5 V và có xung | **H4**: thay SG90, rồi làm lại giai đoạn A cho kẹp (đặt 1500 µs và lắp bánh răng ở khe khoảng 50 mm) trước khi đo B3 |

## 5. Sau khi sửa xong
Nạp lại `pio run -e calib -t upload` và đo kẹp theo bảng B3 trong `docs/hieu_chuan_servo.md`. Nhớ không để hai ngón chạm nhau. Khi xong cả 4 khớp, lắp lại dây OUT+ Buck 2 vào chân 5V và trả CC Buck 1 về 3,0 A như hướng dẫn.

## Nguồn
- `src/calib/main.cpp`, `lib/ServoPwm/`: kênh G dùng chung mã phát xung với J1–J3.
- `mk2_connection_diagram.html` (Project): Hình 1 (Buck 2 nuôi SG90 và chân 5V ESP32), nguồn [7] (màu dây SG90: nâu −, đỏ +, cam/vàng S), mục 5.
- Espressif, ESP32-DevKitC V4 user guide: bảng header J2 (IO33 là chân 8, IO34 là chân 5, chỉ vào), mục Power Supply Options.
- `include/arm_config.h`: dải xung SG90 500–2400 µs theo datasheet.
