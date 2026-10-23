// arm_config.h - thong so tay may lay tu de cuong CE212 (muc "Mo hinh dong hoc",
// "Bo sinh quy dao double-S", "Danh sach phan cung"). Chua duoc dung; danh cho
// code dong hoc / quy dao sau nay. Don vi: mm, do, giay.
#pragma once

namespace armcfg {

// Chieu dai khau, do tren file STL cua tac gia (tam cac lo truc)
constexpr float L_MAIN_ARM      = 135.0f;  // canh tay chinh
constexpr float L_HORIZ_ARM     = 147.0f;  // tay ngang
constexpr float L_HORIZ_TAIL    = 57.0f;   // duoi tay ngang
constexpr float L_CRANK         = 57.0f;   // tay quay
constexpr float L_LINK_A        = 135.0f;  // thanh truyen 1
constexpr float L_LINK_B        = 147.0f;  // thanh truyen 2
constexpr float L_ELBOW_TRI     = 43.0f;   // canh tam giac khuyu
constexpr float BASE_GEAR_RATIO = 50.0f / 25.0f;
constexpr float SHOULDER_HEIGHT = 93.3f;   // TAM: suy ra tu STL, se do lai tren ban in

// Gioi han khop do va cham co khi
constexpr float THETA1_MIN = -45.0f, THETA1_MAX = 45.0f;    // de
constexpr float THETA2_MIN = 35.0f,  THETA2_MAX = 125.0f;   // vai, so voi phuong ngang
constexpr float ELBOW_MIN  = -160.0f, ELBOW_MAX = -70.0f;   // phi - theta2

// Quy dao double-S (theo do dai cung)
constexpr float V_MAX = 150.0f;            // mm/s
constexpr float A_MAX = 1500.0f;           // mm/s^2
constexpr float J_MAX = 30000.0f;          // mm/s^3
constexpr float CONTROL_PERIOD_S = 0.020f; // = chu ky xung servo 20 ms

// Do rong xung SG90 theo datasheet: 500-2400 us. Gioi han MG996R chua co nguon -> hieu chuan.
constexpr unsigned SG90_MIN_US = 500, SG90_MAX_US = 2400;

// Chu trinh phan loai: nha nap o do cao 50 mm tren mieng hop dich.
constexpr float DROP_HEIGHT_ABOVE_BIN = 50.0f;

}  // namespace armcfg
