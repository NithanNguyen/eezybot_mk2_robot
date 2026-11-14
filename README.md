# eezybot_mk2_robot

Firmware ESP32 cho tay máy EEZYbotARM Mk2 (3-DOF) phân loại nắp chai theo màu **xanh lá / đỏ / trắng** – đồ án CE212.
Phần cứng: ESP32-DevKitC-32D (module ESP32-WROOM-32D, PlatformIO `board = esp32dev`), 3 × MG996R, SG90, cảm biến màu GY-33 (TCS34725). Build bằng PlatformIO.

> Tay máy đang ở chế độ **chạy khô**: `src/robot/arm.cpp` tính đủ góc lệnh mỗi nhịp 20 ms (double-S + IK, khớp
> từng nhịp với mô phỏng) nhưng **chưa phát xung servo**. Phát xung cần hiệu chuẩn góc servo → độ rộng xung trước
> (`ARM_OUTPUT_ENABLED=1` hiện báo lỗi biên dịch có chủ ý).

## Cấu trúc

```
platformio.ini        env robot (mặc định) và servo_center
include/
  board_pins.h        phân bổ chân
  arm_config.h        SINH TỰ ĐỘNG từ eezybot_mk2_sim/params.py (tools/export_arm_config.py): hình học,
                      giới hạn khớp, ánh xạ servo, giới hạn quỹ đạo, các điểm của chu trình. Không sửa tay
lib/
  GY33/               driver GY-33, chế độ MCU_IIC (0x5A)
  ColorClassifier/    phân loại màu, lưu model trong NVS
  ServoPwm/           xung servo 50 Hz (LEDC)
  Trajectory/         DoubleS1D.h: double-S 1 chiều nghỉ → nghỉ (float, giải tích), dùng chung cho
                      đường thẳng (s → IK) và chuyển động khớp (σ: 0 → 1)
  Kinematics/         Mk2Kinematics.h: FK, IK giải tích, kiểm tra giới hạn, Jacobian, ánh xạ góc servo
                      (float, tham số từ include/arm_config.h)
  Motion/             ArmMotion.h: đoạn khớp (σ đồng bộ pha + giãn thời gian theo v, a, j của kẹp), đoạn
                      thẳng (s → IK) và bộ tuần tự chu trình phân loại, giống eezybot_mk2_sim/run_sort.py
src/robot/            firmware chính
src/servo_center/     xung 1500 µs cho 4 servo, dùng trước khi lắp horn
src/calib/            hiệu chuẩn servo: điều khiển từng kênh bằng xung, đổi xung từ từ (docs/hieu_chuan_servo.md)
tools/colorcal/       công cụ Python hiệu chuẩn màu
tools/trajectory/     kiểm thử DoubleS1D trên PC so với Ruckig (eezybot_mk2_sim/trajectories.ruckig_1d)
tools/kinematics/     kiểm thử Mk2Kinematics trên PC so với eezybot_mk2_sim/mk2_kinematics.py
tools/motion/         kiểm thử ArmMotion.h và arm.cpp (stub FreeRTOS) từng nhịp so với log.csv của mô phỏng
tools/servo_cal/      tính hiệu chuẩn servo từ số đo bằng thước; kiểm thử logic firmware calib trên PC
tools/host_compat/    bẫy macro Arduino-ESP32 cho các bản biên dịch trên PC
docs/                 quy trình hiệu chuẩn servo
data/color/, reports/color/   dữ liệu và báo cáo hiệu chuẩn
```

## Đấu dây

| Thiết bị | ESP32 |
|---|---|
| MG996R J1 đế / J2 vai / J3 khuỷu | GPIO25 / 26 / 27 |
| SG90 kẹp | GPIO33 |
| GY-33 DR / CT | GPIO21 (SDA) / GPIO22 (SCL), 50 kHz |
| GY-33 VCC / GND | 3V3 / GND |

Trên GY-33, nối jumper **S0–G** để chọn chế độ MCU_IIC; để hở S1 và hàng chân trên.

## Sử dụng

```bash
pip3 install -r tools/colorcal/requirements.txt
pio run -e servo_center -t upload        # 1. căn giữa servo trước khi lắp horn
pio run -e robot -t upload               # 2. nạp firmware chính
pio run -e calib -t upload               #    hiệu chuẩn servo trước khi phát xung: docs/hieu_chuan_servo.md
python3 tools/colorcal/colorcal.py wizard   # 3. hiệu chuẩn màu (đóng Serial Monitor trước)
```

`wizard` chạy lần lượt 7 bước:

1. Kiểm tra kết nối.
2. Chọn mức LED bằng cách quét với nắp trắng.
3. Thu 30 mẫu cho mỗi nhãn: xanh lá, đỏ, trắng, trạm trống.
4. Huấn luyện.
5. Nạp model vào ESP32.
6. Thu tập kiểm tra.
7. Lập ma trận nhầm lẫn.

Kết quả: `reports/color/report.md` và `include/color_model.h`. Nên commit cả `data/color/` để lưu lại lần hiệu chuẩn.

Muốn chạy thử không cần phần cứng, thêm `--sim --yes --ws /tmp/sim`. Số liệu khi đó là giả lập.

Sau khi hiệu chuẩn xong, mở Serial Monitor (115200 baud). Gửi `AUTO 1` để bật chu trình phân loại, `AUTO 0` để tắt, `STATUS` để xem số nắp đã phân loại.

### Tay máy: lệnh Serial và chạy khô

Firmware chạy một task FreeRTOS chu kỳ 20 ms (ưu tiên 3, cùng lõi với `loop()`), nhận yêu cầu qua hàng đợi và tính lệnh mỗi nhịp. Lúc khởi động firmware **coi tay đang ở Home** (θ₁ = 0°, θ₂ = 90°, φ = 0°; servo 90/90/90) vì servo không có phản hồi vị trí.

| Lệnh | Tác dụng |
|---|---|
| `ARM SORT <GREEN\|RED\|WHITE\|REJECT>` | chạy một chu trình gắp → thả → về Home, không cần cảm biến |
| `ARM HOME` | về Home rồi mở kẹp |
| `ARM TRACE 1` / `ARM TRACE 0` | in / tắt mỗi nhịp `CMD,<nhịp>,<pha>,<servo0>,<servo1>,<servo2>,<kẹp>` (độ) |
| `ARM TIMING` / `ARM TIMING RESET` | thời gian tính mỗi nhịp (lớn nhất, trung bình), độ lệch nhịp 20 ms lớn nhất, số nhịp trễ, số dòng trace bị mất |
| `ARM STATUS` | bận hay rảnh, pha, góc lệnh hiện tại |

Kết thúc mỗi chu trình firmware in `ARM DONE <tên> status=<OK|lỗi> ticks=<n> cmax_us=<µs> dropped=<n>`.

Chạy khô trên ESP32 chỉ cần USB, **không cần bật nguồn servo** (S1 tắt). Đóng Serial Monitor rồi chạy:

```bash
pip3 install -r tools/colorcal/requirements.txt     # pyserial
python3 tools/motion/test_motion_parity.py --port /dev/ttyUSB0
```

Script gửi `ARM SORT` cho 4 đích, ghi trace vào `esp32_trace.txt`, so từng nhịp với `log.csv` của mô phỏng và in `ARM TIMING` đo trên ESP32. Có thể ghi trace bằng tay rồi so: `--trace <file>`.

### Kiểm thử bộ sinh quỹ đạo, động học và chuyển động trên PC

Cần `g++` và repo [eezybot_mk2_sim](https://github.com/NithanNguyen/eezybot_mk2_sim) clone cạnh repo này.

```bash
pip3 install -r tools/trajectory/requirements.txt
python3 tools/trajectory/test_doubles_parity.py              # hoặc --sim <đường dẫn eezybot_mk2_sim>
```

Script biên dịch `tools/trajectory/host_doubles.cpp` (kèm `-Wdouble-promotion -Werror` để chặn phép tính `double`). Sau đó nó so từng mẫu s(t), v(t), a(t) và thời gian T với Ruckig trên 2029 ca: đường thẳng thật của chu trình, chuyển động khớp σ, 2000 ca ngẫu nhiên phủ cả 4 trường hợp (đạt hoặc không đạt v_max, a_max) và các ca biên. Kết quả đạt khi dòng cuối kết thúc bằng `0 khac biet.` và mã thoát là 0.

```bash
python3 tools/kinematics/test_kinematics_parity.py           # chỉ cần numpy; hoặc --sim <đường dẫn>
```

Script này kiểm tra trước rằng các hằng số động học trong `include/arm_config.h` trùng `params.py`. Nếu không trùng, cần chạy lại `tools/export_arm_config.py` của mô phỏng. Sau đó nó so `Mk2Kinematics.h` (float) với `mk2_kinematics.py` (double) trên khoảng 180 000 lệnh: FK, Jacobian, ánh xạ servo hai chiều, `withinLimits` và IK. Phần IK gồm điểm lấy từ FK, điểm ngẫu nhiên (nhiều điểm ngoài tầm với), điểm sát trục đế và 7 điểm của chu trình. Với IK, script so cả trạng thái (OK, NEAR_AXIS, OUT_OF_REACH hay giới hạn nào bị vi phạm) lẫn góc. Ca nằm sát ngưỡng giới hạn (dưới 0,002°) không so trạng thái vì làm tròn float có thể lật kết quả so sánh, nhưng vẫn được đếm và in ra.

```bash
python3 tools/motion/test_motion_parity.py                  # g++ có -pthread; hoặc --sim <đường dẫn>
```

Script so lệnh của firmware **từng nhịp 20 ms** với `eezybot_mk2_sim/results/sort/log.csv` (11 chu trình gồm cả 4 đích): số nhịp và tên pha phải trùng, góc 3 servo và góc kẹp lệch ≤ 0,005° (nhỏ hơn 1 bước LEDC 16 bit ≈ 0,0275°), vận tốc kẹp ≤ 150 mm/s. Nguồn [1] chỉ chạy `lib/Motion/ArmMotion.h`; nguồn [2] chạy chính `src/robot/arm.cpp` (task, hàng đợi, lệnh `ARM`, đường in Serial) với stub FreeRTOS trong `tools/motion/stub`.

Mọi chương trình kiểm thử trên PC include `tools/host_compat/arduino_macro_trap.h` sau header hệ thống và trước `lib/`. File này định nghĩa lại 383 macro của Arduino-ESP32 (`cores/esp32/*.h`) và newlib `<ctype.h>` (`_U _L _N _S _P _C _X _B`) thành hằng số. Nếu code trong `lib/` đặt tên trùng một macro, PC báo lỗi giống ESP32, ví dụ biến `_L` trùng macro `_L` của newlib. Khi đổi phiên bản Arduino-ESP32, sinh lại bẫy từ framework đã cài:

```bash
python3 tools/host_compat/gen_macro_trap.py ~/.platformio/packages/framework-arduinoespressif32/cores/esp32
```

## Phân loại màu

- Mỗi phép đo chờ 300 ms, sau đó lấy trung bình 5 lần đọc RGBC thô từ thanh ghi 0x00–0x07, cách nhau 120 ms.
- Tính sắc độ r = R/(R+G+B), g = G/(R+G+B), rồi gán màu theo tâm lớp gần nhất (khoảng cách Euclid).
- Nếu khoảng cách tới tâm gần nhất lớn hơn d_max = max(d̄ + 3σ), nắp được đưa vào khu vực **vứt bỏ** và firmware báo lỗi qua Serial.
- Nếu |C − C_empty| < C_margin, coi như **không có nắp** và tiếp tục chờ.

GY-33 không cho chỉnh thời gian tích phân và độ lợi; chỉ chỉnh được mức LED (0–10, số nhỏ là sáng hơn). Firmware lưu mức LED trong model và tự đặt lại sau mỗi lần khởi động.

Tài liệu tham khảo:

- Đề cương CE212 và sơ đồ `mk2_connection_diagram`.
- Tài liệu GY-33 V1.0 của hãng và mã mẫu `arduino_iic.ino`.
- T. Gevers, A. Smeulders, *Color-based object recognition*, Pattern Recognition 32 (1999).
