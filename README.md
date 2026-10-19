# eezybot_mk2_robot

Firmware và công cụ cho tay máy EEZYbotARM Mk2 (3-DOF) phân loại nắp chai theo màu – đề tài CE212.
ESP32 NodeMCU-32S · 3 × MG996R + SG90 · cảm biến màu TCS34725 · PlatformIO.

| Thành phần | Trạng thái |
|---|---|
| Căn giữa servo trước khi lắp horn (`servo_center`) | Xong |
| Đọc TCS34725, phân loại màu, hiệu chuẩn tự động (`tools/colorcal`) | Xong |
| Chu trình phân loại: chờ nắp → quyết định hộp / vứt bỏ | Xong (logic) |
| Điều khiển tay máy: Home, FK/IK, quỹ đạo, gắp – thả | **Chưa** – khung trong `src/robot/arm.*` |

## Cấu trúc thư mục

```
eezybot_mk2_robot/
├── platformio.ini            2 môi trường build: robot (mặc định), servo_center
├── include/
│   ├── board_pins.h          toàn bộ phân bổ chân ESP32 (một nơi duy nhất)
│   └── color_model.h         (sinh khi hiệu chuẩn màu) model biên dịch sẵn, dự phòng
├── lib/                      thư viện dùng chung giữa các chương trình
│   ├── ColorClassifier/      đọc TCS34725, phân loại màu, lưu model trong NVS
│   │   └── ColorMath.h       thuật toán thuần C++ (kiểm thử được trên PC)
│   └── ServoPwm/             xung servo 50 Hz bằng LEDC (core 2.x và 3.x)
├── src/
│   ├── robot/                FIRMWARE CHÍNH
│   │   ├── main.cpp          khởi tạo + console Serial
│   │   ├── color_console.*   lệnh cảm biến màu (dùng cho hiệu chuẩn)
│   │   ├── sorter.*          chu trình phân loại
│   │   └── arm.*             điều khiển tay máy (khung, chưa phát xung)
│   └── servo_center/         chương trình phụ: 1500 µs cho 4 servo
├── tools/colorcal/           Python: hiệu chuẩn, đánh giá, báo cáo
├── data/color/               dữ liệu hiệu chuẩn (CSV, config.json, model.json) – nên commit
└── reports/color/            báo cáo + hình do colorcal sinh
```

Quy ước khi mở rộng:
- Mã dùng chung hoặc kiểm thử được trên PC (động học FK/IK, bộ sinh quỹ đạo double-S…) → `lib/<TênThưViện>/`.
- Mã chỉ của firmware chính (máy trạng thái, cấu hình tay máy) → `src/robot/`.
- Chương trình thử nghiệm độc lập mới → `src/<tên>/` và thêm một `[env:<tên>]` với `build_src_filter = +<tên>/`.
- Đổi chân → chỉ sửa `include/board_pins.h`.

## Phần cứng

Theo sơ đồ đấu dây `mk2_connection_diagram`:

| Thiết bị | Chân ESP32 |
|---|---|
| MG996R J1 đế / J2 vai / J3 khuỷu | GPIO25 / GPIO26 / GPIO27 |
| SG90 kẹp | GPIO33 |
| TCS34725 SDA / SCL / LED | GPIO21 / GPIO22 / GPIO23 (HIGH = bật) |
| TCS34725 VIN | 3V3 |

## Bắt đầu (Ubuntu 20.04, PlatformIO trên VS Code)

```bash
git clone https://github.com/<user>/eezybot_mk2_robot.git
cd eezybot_mk2_robot
pip3 install -r tools/colorcal/requirements.txt
sudo usermod -aG dialout $USER          # một lần, rồi đăng xuất/đăng nhập lại
```

Mở thư mục repo trong VS Code. Trên thanh trạng thái PlatformIO, chọn môi trường
(`env:robot` hoặc `env:servo_center`) rồi Upload; hoặc dùng dòng lệnh như bên dưới.
`upload_port` mặc định là `/dev/ttyUSB0` (sửa trong `platformio.ini` nếu khác).

### 1. Căn giữa servo (trước khi lắp horn)
```bash
pio run -e servo_center -t upload
```

### 2. Nạp firmware chính
```bash
pio run -e robot -t upload
```
Tay máy **chưa** phát xung servo (`ARM_OUTPUT_ENABLED = 0` trong `src/robot/arm.h`) cho tới khi có Home và động học.

### 3. Hiệu chuẩn cảm biến màu (chạy trên chính firmware `robot`, không cần nạp chương trình khác)
Đóng Serial Monitor rồi chạy:
```bash
python3 tools/colorcal/colorcal.py wizard
```
7 bước có hướng dẫn: kiểm tra kết nối → tự chọn thời gian tích phân/độ lợi → thu 30 mẫu × {xanh lá, đỏ, vàng, trạm trống}
→ tính tâm lớp, d_max, ngưỡng kênh C → nạp model vào NVS → thu tập kiểm tra độc lập → ma trận nhầm lẫn + kiểm tra ESP32 cho cùng kết quả với Python.

Kết quả ở `reports/color/report.md` (có đoạn văn mô tả phương pháp điền sẵn số liệu cho báo cáo) và `include/color_model.h`.
Commit `data/color/`, `reports/color/` và `include/color_model.h` để lưu lại lần hiệu chuẩn.

Lệnh lẻ: `ports | check | tune | collect | train | push | evaluate | live | report` – xem
`python3 tools/colorcal/colorcal.py -h`. Chạy thử không cần phần cứng: thêm `--sim --yes --ws /tmp/sim`
(số liệu mô phỏng, không dùng cho báo cáo).

### 4. Chạy phân loại
Mở Serial Monitor (115200), gửi `AUTO 1` để bật, `AUTO 0` để tắt.

| Kết quả | Hành động |
|---|---|
| Không có nắp (`\|C − C_empty\| < C_margin`) | tiếp tục chờ |
| Xanh lá / đỏ / vàng | gắp vào hộp tương ứng |
| Không xác định (khoảng cách tới tâm gần nhất > d_max) | gắp vào khu vực **vứt bỏ**, in `# ERR UNKNOWN_COLOR …` |

`AUTO 1` bị từ chối nếu chưa có cảm biến hoặc model. Phần gắp – thả hiện chỉ in `# arm TODO …`.

## Lệnh Serial của firmware `robot`

| Lệnh | Mô tả |
|---|---|
| `AUTO 0\|1`, `STATUS` | bật/tắt chu trình; trạng thái và số nắp theo từng hộp |
| `PING`, `INFO` | kiểm tra kết nối, cấu hình cảm biến |
| `CFG <atime> <gain>`, `LED 0\|1` | cấu hình TCS34725, bật/tắt LED |
| `MEAS <navg> <nflush>` | đo RGBC trung bình |
| `CLS`, `CLSRAW R G B C sat` | phân loại một lần / phân loại số liệu cho trước |
| `MODEL GET\|SET\|CLEAR` | đọc / ghi (NVS) / xoá model màu |
| `RUN <ms>` | phân loại liên tục, `RUN 0` để dừng |

## Kiểm thử trên PC

```bash
python3 tools/colorcal/colorcal.py --sim --yes --ws /tmp/sim wizard     # toàn bộ quy trình, thiết bị mô phỏng
python3 tools/colorcal/test_host_parity.py --ws /tmp/sim                # ColorMath.h (g++) == Python
pio run -e robot -e servo_center                                         # build cả 2 env
```

## Phương pháp phân loại màu và căn cứ

Chuẩn hóa sắc độ r = R/(R+G+B), g = G/(R+G+B); gán màu theo tâm lớp gần nhất (Euclid, tâm = trung bình mẫu);
d_max = max_k(d̄_k + 3σ_k); mức bão hòa kênh C theo logic DN40 trong thư viện Adafruit.

| Lựa chọn | Căn cứ |
|---|---|
| Chuẩn hóa (r, g) | Màu chuẩn hóa rgb bất biến với cường độ chiếu sáng và hình học vật thể khi bề mặt nhám, nguồn sáng trắng [1] |
| Tâm lớp gần nhất | [2] |
| Mức bão hòa, bỏ 3 lần đọc đầu sau khi bật LED | `getRawData()` đọc thanh ghi rồi mới chờ một chu kỳ tích phân; logic bão hòa trong `calculateColorTemperature_dn40()` [3] |
| 60 % bão hòa khi chọn cấu hình, K = 3 | Lựa chọn kỹ thuật, chưa có nguồn riêng; chỉnh được (`TUNE_HEADROOM`, `--k-sigma`) |

[1] T. Gevers, A. W. M. Smeulders, "Color-based object recognition", *Pattern Recognition* 32 (1999) 453–464.
[2] scikit-learn, `NearestCentroid`: https://scikit-learn.org/stable/modules/generated/sklearn.neighbors.NearestCentroid
[3] Adafruit_TCS34725 v1.4.4: https://github.com/adafruit/Adafruit_TCS34725
