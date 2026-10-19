// board_pins.h - phan bo chan ESP32 NodeMCU-32S, theo mk2_connection_diagram (muc 3).
// Moi chuong trinh trong src/ lay chan tu day; doi day thi chi sua file nay.
#pragma once
#include <stdint.h>

namespace pins {

// Servo (PWM 50 Hz qua LEDC)
constexpr uint8_t SERVO_J1_BASE     = 25;  // MG996R J1 de
constexpr uint8_t SERVO_J2_SHOULDER = 26;  // MG996R J2 vai
constexpr uint8_t SERVO_J3_ELBOW    = 27;  // MG996R J3 khuyu
constexpr uint8_t SERVO_GRIPPER     = 33;  // SG90 kep

// Cam bien mau TCS34725 (I2C 0x29)
constexpr uint8_t I2C_SDA   = 21;
constexpr uint8_t I2C_SCL   = 22;
constexpr uint8_t COLOR_LED = 23;          // HIGH = bat LED cua module

}  // namespace pins
