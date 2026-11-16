# Chạy tay máy thật lần đầu

Tài liệu này nối tiếp `docs/hieu_chuan_servo.md`. Từ bản này env `robot` **phát xung servo thật**: mỗi nhịp 20 ms, góc khớp được đổi sang độ rộng xung theo số hiệu chuẩn trong `eezybot_mk2_sim/params.py` (mục 4, `SERVO_PULSE_*`), rồi kẹp vào 500–2500 µs.

## 0. Trạng thái hiện tại

| Phần | Trạng thái | Hệ quả |
|---|---|---|
| Đế, vai, khuỷu (MG996R) | Đã hiệu chuẩn (`[DO-THAT]`, sai số dư 0,45–0,58°) | Firmware phát xung cho GPIO25, 26, 27 |
| Kẹp (SG90) | Servo hỏng, số trong `params.py` là số giả định (`GRIP_PULSE_MEASURED = False`) | Firmware **không** phát xung GPIO33. Tay máy chạy đúng quỹ đạo nhưng **không gắp được nắp** |
| Mô hình màu | `include/color_model.h` có sẵn | Board mới chưa có model trong NVS nên firmware dùng model biên dịch sẵn này |

Do đó hiện tại chỉ chạy được **chuyển động của chu trình** (`ARM SORT`). Muốn phân loại nắp thật phải làm thêm mục 5.

## 1. Kiểm thử trên PC (không cần phần cứng)

```bash
python3 tools/kinematics/test_kinematics_parity.py   # gồm cả độ rộng xung C++ so với mk2_kinematics.to_pulse()
python3 tools/motion/test_motion_parity.py           # chu trình từng nhịp so với mô phỏng
```
Cả hai phải kết thúc bằng `0 khac biet.`

## 2. Kiểm tra từng tư thế thật chậm bằng firmware `calib`

Firmware `robot` đi hết chu trình với tốc độ đầu kẹp tới 150 mm/s. Vì vậy trước tiên bạn đưa tay tới từng tư thế bằng `calib` với tốc độ đổi xung 50 µs/s. Mục đích là phát hiện sớm chiều quay sai hoặc va chạm khi tay còn đi chậm.

Chuẩn bị an toàn giống bước B0 của `hieu_chuan_servo.md`: tắt S1, **tháo dây OUT+ Buck 2 khỏi chân 5V**, hạ CC Buck 1 về khoảng 1,0 A, cắm USB, nạp `pio run -e calib -t upload`, mở monitor, bật S1.

```
RATE 50
LIM J1 600 2400
LIM J2 1400 2150
LIM J3 700 1550
ON J1
ON J2
ON J3 1490
```

Xung của từng tư thế, tính bằng `mk2_kinematics.to_pulse()` với số hiệu chuẩn hiện tại (điểm công tác là tâm nắp):

| Tư thế | p (mm) | Góc khớp θ₁ / θ₂ / φ (°) | Xung J1 / J2 / J3 (µs) |
|---|---|---|---|
| Home | — | 0 / 90 / 0 | 1500 / 1500 / 1490 |
| Trạm: phía trên | (279; 0; 99) | −0,10 / 57,94 / −48,54 | 1503 / 1906 / 925 |
| Trạm: điểm kẹp | (279; 0; 59) | −0,10 / 43,50 / −61,09 | 1503 / 2088 / 779 |
| Ngăn 1 đỏ: thả | (280,6; −162; 116) | −30,08 / 48,09 / −32,61 | 2327 / 2030 / 1111 |
| Ngăn 2 xanh lá: thả | (230,4; −133; 116) | −30,10 / 67,52 / −44,75 | 2328 / 1784 / 969 |
| Ngăn 3 trắng: thả | (180,1; −104; 116) | −30,13 / 87,97 / −50,65 | 2328 / 1526 / 901 |
| Vứt bỏ: phía trên | (246,8; 142,5; 99) | 29,90 / 56,17 / −47,22 | 678 / 1928 / 941 |
| Vứt bỏ: thả | (246,8; 142,5; 66) | 29,90 / 44,79 / −57,41 | 678 / 2072 / 822 |

**Thứ tự đổi xung** để tay không quét xuống mặt bàn: khi đi ra, đổi **J3 trước**, rồi J2, rồi J1. Khi về Home thì làm ngược lại (J1, J2, rồi J3). Ví dụ từ Home tới "Trạm: phía trên":
```
SEL J3
US 925
SEL J2
US 1906
SEL J1
US 1503
```

Ở mỗi tư thế, kiểm tra:
- **Chiều quay.** Ngăn hộp nằm bên **phải** robot (y âm) khi đứng sau robot nhìn ra trước. Vùng vứt bỏ nằm bên **trái**. Nếu đế quay ngược thì dấu của dữ liệu `chord` bị ghi ngược: dừng lại và đo lại J1.
- **Vị trí.** Tâm giữa hai ngón kẹp phải nằm gần đúng điểm p. Ví dụ ở "Trạm: phía trên", tâm kẹp ở ngay trên tâm kệ và cao hơn mặt kệ 56 mm (99 − 43). Lệch vài mm là bình thường vì đế có độ rơ và cánh tay bị võng.
- **"Trạm: điểm kẹp" làm sau cùng.** Đây là tư thế sát giới hạn nhất (θ₂ cách cận dưới 1,0°). Mô phỏng còn cho thấy tay võng làm ngón kẹp chỉ cách mặt kệ khoảng 3 mm (đề cương mục 6.2). Đi từ "phía trên" xuống bằng các bước nhỏ và dừng ngay nếu ngón chạm kệ.

Xong thì đưa tay về Home theo thứ tự ngược lại, gõ `STOP` rồi tắt S1.

## 3. Nạp firmware `robot` và về Home

1. Tắt S1. Dây OUT+ Buck 2 **vẫn tháo** vì bạn cần USB để gửi lệnh.
2. **Đặt tay về gần Home bằng tay.** Firmware không có cảm biến vị trí, nên ngay khi khởi động nó phát xung Home (1500 / 1500 / 1490 µs). Nếu tay đang ở xa Home, servo sẽ giật về Home với tốc độ tối đa.
3. Cắm USB, nạp `pio run -e robot -t upload`, mở monitor. Phải thấy hai dòng:
   ```
   # arm: task 20 ms, ARM_OUTPUT_ENABLED=1 (phat xung)
   # arm: kep CHUA hieu chuan (GRIP_PULSE_MEASURED = false) -> khong phat xung GPIO33
   ```
4. Gõ `ARM STATUS`. Dòng trả về phải có `output=1` và `us=1500/1500/1490/...`. Số thứ tư là xung kẹp tính ra, nhưng chưa được phát.
5. Bật S1. Tay giữ ở Home.

**Dừng khẩn cấp:** gõ `ARM OFF` (ngừng xung, servo thả lỏng) hoặc tắt S1. Sau `ARM OFF`, đỡ tay về Home rồi nhấn **EN** để bật lại. Lệnh này chốt, không tự bật lại được bằng lệnh khác.

## 4. Chạy chuyển động của chu trình (chưa có nắp)

Dọn trạm, hộp và vùng vứt bỏ, để tay gần S1. Gõ lần lượt và chờ `ARM DONE <tên> status=OK` sau mỗi lệnh:
```
ARM SORT WHITE
ARM SORT GREEN
ARM SORT RED
ARM SORT REJECT
ARM TIMING
```
Ngăn trắng gần robot nhất nên chạy trước. `ARM TIMING` phải có `sat=0` (không xung nào bị kẹp vào 500–2500 µs) và `late=0`.

Khi đã ổn ở CC 1,0 A, **trả CC Buck 1 về 3,0 A** rồi chạy lại 4 lệnh trên. Theo sơ đồ đấu nối (mục 4 bước 9), lúc chạy thì áp OUT Buck 1 phải ≥ 4,8 V và đèn giới hạn dòng không được sáng liên tục.

## 5. Để phân loại nắp thật (cần SG90 mới)

1. Thay SG90. Làm giai đoạn A cho kẹp (1500 µs, lắp bánh răng ở khe khoảng 50 mm), sau đó đo B3 cho kẹp.
2. Chạy `fit_servo_cal.py`. Dán `GRIP_PULSE_CLOSED_US`, `GRIP_PULSE_PER_DEG` vào `eezybot_mk2_sim/params.py`, đổi nhãn thành `[DO-THAT]` và đặt `GRIP_PULSE_MEASURED = True`.
3. `python tools/export_arm_config.py` trong repo mô phỏng, rồi nạp lại `pio run -e robot -t upload`.
4. Kiểm tra cảm biến màu bằng vài nắp đặt trên trạm (lệnh `MEAS`, `CLS`: xem `color_console.h`). Nếu nhận sai thì chạy lại `python3 tools/colorcal/colorcal.py wizard`.
5. `AUTO 1`, đặt từng nắp lên trạm. Đề cương yêu cầu 25 chu trình không có lỗi nghiêm trọng.

Hiện tại `AUTO` mặc định tắt khi khởi động, nên bật nó cần USB. Muốn chạy không có USB (lắp lại dây OUT+ Buck 2 vào chân 5V) thì firmware phải có cách bật `AUTO` mà không cần lệnh Serial. Chức năng này chưa có.

## Nguồn
- `eezybot_mk2_sim/params.py` mục 4 (`SERVO_PULSE_*`, `GRIP_PULSE_*`, `MG996R_PULSE_US`); `sort_scene.task_points()` (các điểm p).
- `include/arm_config.h` (sinh từ `params.py`); `src/robot/arm.cpp` (`pulses()`, `output()`, lệnh `ARM OFF`).
- `docs/hieu_chuan_servo.md` (an toàn nguồn, CC Buck 1, lệnh `calib`).
- `mk2_connection_diagram.html` (Project): mục 4 bước 9, mục 5.
- Đề cương CE212 mục 5.8 (25 chu trình) và mục 6.2 (độ võng tại điểm kẹp).
