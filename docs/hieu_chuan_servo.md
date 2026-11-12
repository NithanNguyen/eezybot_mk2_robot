# Hiệu chuẩn servo (bước 6)

Mục tiêu là tìm, cho từng servo, **độ rộng xung ứng với tư thế Home** và **hệ số µs trên mỗi độ khớp (có dấu)**. Hiện ba thông tin này trong `params.py` mới là giả định `[GIA-DINH]`, vì vậy firmware chính đang chặn phát xung (`ARM_OUTPUT_ENABLED=1` báo lỗi biên dịch). Khi có số đo thật, chúng được ghi vào `params.py` rồi xuất sang `include/arm_config.h`. Sau đó firmware mới đổi được góc khớp thành xung.

Quy trình chỉ cần thước thẳng chia mm, dây dọi (sợi chỉ buộc một đai ốc), giấy A4, băng dính, bút, đồng hồ vạn năng và tua vít.

Tư thế Home (`params.py` mục 4): θ₁ = 0° (tay hướng thẳng ra trước, đây chính là trục +x của bố trí trạm và hộp), θ₂ = 90° (cánh tay chính thẳng đứng), φ = 0° (tay ngang nằm ngang).

## 0. An toàn nguồn

1. **USB và S1 cùng lúc.** Sơ đồ đấu nối (mục 5, "Nạp firmware qua USB") cấm bật S1 khi đang cắm USB, vì chân 5V của ESP32 nối với ngõ ra Buck 2 và chưa rõ board có diode chống dòng ngược. Giai đoạn B cần cả hai: USB để gửi lệnh, S1 để cấp nguồn servo. Vì vậy trước giai đoạn B, **tháo dây OUT+ của Buck 2 khỏi chân 5V của ESP32**, theo đúng cách sơ đồ đề xuất. Khi đó ESP32 lấy nguồn từ USB. Giữ nguyên mọi dây GND. Lắp lại dây này khi hiệu chuẩn xong.
2. **Giới hạn dòng (khuyến nghị của mình, không có trong sơ đồ).** Trong lúc hiệu chuẩn, hạ CC của Buck 1 xuống khoảng 1,0 A theo cách chỉnh CC ở sơ đồ mục 4 bước 2 (chỉnh CV trước, rồi chạm đồng hồ thang 10 A vào OUT+/OUT−). Nếu servo giật thì lực sẽ nhỏ. Sơ đồ ghi dòng chạy của MG996R khoảng 0,5–0,9 A mỗi con. Nếu servo yếu, không giữ nổi tay, tăng dần lên. **Trả CC về 3,0 A** khi xong.
3. Dọn trạm, hộp và mọi vật quanh tay máy. Luôn để tay gần công tắc S1, và gõ `STOP` hoặc tắt S1 ngay khi có tiếng rè lớn hay khớp chạm vật.

## A. Đặt servo về 1500 µs rồi lắp lại horn ở Home

1. Tắt S1. Tháo horn của vai (J2) và khuỷu (J3). Gỡ bánh răng đế (J1) khỏi ăn khớp và tháo bánh răng kẹp (SG90) để trục servo quay tự do, không kéo theo cơ cấu.
2. Cắm USB (S1 vẫn tắt), nạp `pio run -e servo_center -t upload`, rồi **rút USB**. Bật S1. Cả 4 servo về 1500 µs. Chờ 2 s rồi tắt S1.
3. Lắp lại, chọn vị trí răng then gần Home nhất:
   - **J2 vai:** cánh tay chính thẳng đứng. Kiểm bằng dây dọi treo từ tâm trục khuỷu E: dây phải đi qua tâm trục vai O.
   - **J3 khuỷu:** tay ngang nằm ngang. Đo độ cao tâm trục E và tâm trục cổ tay W so với mặt bàn, hai số phải bằng nhau.
   - **J1 đế:** tay hướng thẳng ra trước, theo hướng sẽ đặt trạm cảm biến.
   - **Kẹp:** mở khoảng nửa hành trình, khe giữa hai mặt kẹp khoảng 50 mm. Khi đó còn chỗ để đóng và mở. Theo hình học trong mô phỏng, khe 50 mm ứng với góc bánh răng 45°.
   
   Răng then không cho đặt đúng tuyệt đối, nên lệch vài độ là bình thường. Giai đoạn B đo đúng phần lệch này.

## B. Đo

### B0. Chuẩn bị
1. Tắt S1. **Tháo dây OUT+ Buck 2 khỏi chân 5V ESP32** (mục 0.1).
2. Cắm USB, nạp `pio run -e calib -t upload`, mở `pio device monitor` (115200). Màn hình phải hiện `# READY calib`. Cả 4 kênh đều đang tắt.
3. Tháo dây tín hiệu S của các servo **không** đo ở bước đang làm.
4. Bật S1.

### B1. Lần cấp xung đầu tiên cho mỗi servo
Gõ `ON J2` (hoặc `ON J1`, `ON J3`, `ON G`). Servo nhận ngay 1500 µs. Vì giai đoạn A đã để servo ở 1500 µs, nó gần như không được nhảy. Nếu khớp giật mạnh hay rè liên tục, gõ `STOP`, tắt S1, rồi kiểm tra lại bước A3.

**Giữ các khớp khác.** Khi một MG996R không có xung, mình chưa có tài liệu xác nhận nó có giữ vị trí hay không. Nếu tay ngang hay cánh tay chính bị rũ xuống khi chỉ một servo có xung, hãy nối lại dây S của các servo kia và gõ `ON J1`, `ON J2`, `ON J3` (đều 1500 µs) để chúng giữ Home. Sau đó chọn khớp cần đo bằng `SEL <kênh>`.

### B2. Tìm chiều quay
`SEL <kênh>`, rồi gõ `+ 50`, quan sát chiều khớp quay, rồi `- 50` để về lại. Ghi lại chiều quay. Bước này quyết định nên đi về phía nào ở bước B3. Tốc độ mặc định là 100 µs/s; gõ `RATE 50` nếu muốn chậm hơn. Firmware chỉ cho xung trong khoảng 1000–2000 µs (kẹp: 900–2100 µs). Muốn đổi thì dùng `LIM <kênh> <lo> <hi>`.

### B3. Đo tại 5 mức xung cho mỗi khớp
Mỗi lần gõ `US <µs>`, đợi dòng `AT <kênh> <µs>`, rồi mới đo. Ghi mỗi lần đo thành một dòng trong CSV (mẫu: `tools/servo_cal/measurements_template.csv`).

| Khớp | Mức xung đề xuất | Đo gì (cột `method`, `value`) |
|---|---|---|
| **J2 vai** (`shoulder`) | 1300, 1400, 1500, 1600, 1700 | `cos`: treo dây dọi từ tâm trục khuỷu E, đo **khoảng cách ngang** (mm) từ dây tới tâm trục vai O. Ghi số **dương** nếu E ở phía trước O (về phía trạm), **âm** nếu ở phía sau |
| **J3 khuỷu** (`elbow`) | 1500, rồi 4 mức theo chiều làm tay ngang **chúc xuống** (±100, ±200, ±300, ±400 tùy chiều ở B2). Theo chiều ngẩng lên chỉ đi tối đa 100 µs, vì giới hạn tầm lệnh là φ ≤ 17,5° và va chạm cơ khí ở khoảng 22,5° | `sin`: độ cao tâm trục cổ tay W **trừ** độ cao tâm trục khuỷu E (mm), đo bằng thước dựng đứng trên bàn. Dương nếu W cao hơn E |
| **J1 đế** (`base`) | 1100, 1300, 1500, 1700, 1900 | `chord`: dán giấy dưới đầu kẹp. Ở 1500 µs, thả dây dọi từ điểm giữa hai đầu ngón và đánh dấu điểm gốc. Ở mỗi mức khác, đánh dấu điểm mới, đo **dây cung** (mm) từ điểm gốc tới điểm mới. Ghi **dương** nếu điểm mới lệch sang **trái** khi bạn đứng sau robot nhìn ra trước. Cột `radius` là khoảng cách ngang từ tâm trục đế tới điểm gốc (mm), cùng một số cho mọi dòng. Giữ nguyên lệnh kẹp trong suốt lần đo này |
| **Kẹp** (`gripper`) | 5 mức sao cho khe từ khoảng 10 tới 60 mm. Tìm chiều đóng ở B2, và **không để hai ngón chạm nhau** | `gap`: khe nhỏ nhất giữa hai mặt kẹp (mm) |

Vì sao các mức xung trên an toàn và đủ chính xác:
- Với giả định 11 µs/độ (2000 µs cho 180°), ±200 µs ở vai là khoảng ±18°. Vậy θ₂ nằm trong 72–108°, vẫn trong giới hạn 42,5–125°.
- ±400 µs ở đế là khoảng ±36° servo, tức ±18° khớp (tỉ số truyền 2), trong giới hạn ±45°. Với bán kính khoảng 285 mm, dây cung dài khoảng 2·285·sin 9° ≈ 89 mm.
- Đo bằng thước sai ±1 mm cho sai số góc cỡ 0,4° ở vai và khuỷu. Ở đế, với dây cung dài cỡ 90 mm, sai số tương đối khoảng 1%.
- Script `--selftest` mô phỏng đúng mức sai số này: xung Home lệch dưới 1 µs và hệ số k lệch dưới 0,3%.

Đo xong một khớp, gõ `OFF <kênh>` (nếu không cần nó giữ các khớp khác), rồi chuyển sang khớp tiếp theo. Xong cả 4 khớp, gõ `STOP`, tắt S1, **lắp lại dây OUT+ Buck 2** và **trả CC Buck 1 về 3,0 A**.

## C. Tính và gửi kết quả

```bash
python3 tools/servo_cal/fit_servo_cal.py do_dac.csv      # can numpy, scipy, trimesh (requirements cua eezybot_mk2_sim)
```

Script in ra, cho mỗi khớp:
- xung tại Home;
- hệ số k (µs/độ khớp, dấu cho biết chiều quay);
- sai số dư (độ);
- dải xung tương ứng với giới hạn khớp.

Script cảnh báo khi sai số dư lớn hơn 1° (nên đo lại) hoặc khi giới hạn khớp đòi xung ngoài 500–2500 µs. Cuối cùng nó in sẵn đoạn `SERVO_PULSE_HOME_US`, `SERVO_PULSE_PER_DEG`, `GRIP_PULSE_CLOSED_US` và `GRIP_PULSE_PER_DEG` để dán vào `params.py`.

Gửi lại **file CSV và toàn bộ output** của script. Bước tiếp theo sẽ là:
1. Đưa các số này vào `params.py` và xuất lại `arm_config.h`.
2. Thêm ánh xạ góc → xung vào `arm.cpp` và bỏ chặn `ARM_OUTPUT_ENABLED`.
3. Chạy lần đầu với tốc độ thấp: chỉ `ARM HOME`, rồi tới phía trên trạm, trước khi chạy cả chu trình.

## Kiểm thử trên PC

```bash
python3 tools/servo_cal/test_calib_host.py                # logic firmware calib (Arduino-ESP32 core 2.x va 3.x)
python3 tools/servo_cal/fit_servo_cal.py --selftest       # script tinh voi du lieu gia co nhieu
```

## Nguồn
- `mk2_connection_diagram.html` (Project): mục 4 bước 2 (chỉnh CV/CC của XL4015), mục 5 (USB và S1, dây OUT+ Buck 2), bảng nguồn (dòng MG996R 0,5–0,9 A).
- `eezybot_mk2_sim/params.py`: Home, `L_UPPER` = 135 mm, `L_FORE` = 147 mm, giới hạn khớp và tầm lệnh, `GRIP_*`.
- `eezybot_mk2_sim/gripper.py`: `gap_at()`, khe giữa hai ngón theo góc bánh răng kẹp, dựng từ ảnh gripper thật.
- `include/arm_config.h`: SG90 500–2400 µs (datasheet).
