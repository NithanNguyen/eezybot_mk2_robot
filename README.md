# eezybot_mk2_robot

Firmware ESP32 cho tay máy EEZYbotARM Mk2 (3-DOF) phân loại nắp chai theo màu **xanh lá / đỏ / trắng** – đồ án CE212.
Phần cứng: ESP32 NodeMCU-32S, 3 × MG996R, SG90, cảm biến màu GY-33 (TCS34725). Build bằng PlatformIO.

> Điều khiển tay máy (Home, FK/IK, quỹ đạo double-S) **chưa có**: `src/robot/arm.*` hiện chỉ là khung và chưa phát xung servo.

## Cấu trúc

```
platformio.ini        env robot (mặc định) và servo_center
include/
  board_pins.h        phân bổ chân
  arm_config.h        thông số động học, giới hạn khớp, giới hạn quỹ đạo (theo đề cương)
lib/
  GY33/               driver GY-33, chế độ MCU_IIC (0x5A)
  ColorClassifier/    phân loại màu, lưu model trong NVS
  ServoPwm/           xung servo 50 Hz (LEDC)
src/robot/            firmware chính
src/servo_center/     xung 1500 µs cho 4 servo, dùng trước khi lắp horn
tools/colorcal/       công cụ Python hiệu chuẩn màu
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
