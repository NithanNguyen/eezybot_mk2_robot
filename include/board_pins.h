// board_pins.h - phan bo chan ESP32 NodeMCU-32S, theo mk2_connection_diagram (muc 3).
// Doi day thi chi sua file nay.
#pragma once
#include <stdint.h>

namespace pins {

// Servo (PWM 50 Hz qua LEDC)
constexpr uint8_t SERVO_J1_BASE     = 25;  // MG996R J1 de
constexpr uint8_t SERVO_J2_SHOULDER = 26;  // MG996R J2 vai
constexpr uint8_t SERVO_J3_ELBOW    = 27;  // MG996R J3 khuyu
constexpr uint8_t SERVO_GRIPPER     = 33;  // SG90 kep

// Cam bien mau GY-33, che do MCU_IIC (pad S0 noi G), dia chi 0x5A.
// Module khong co chan dieu khien LED; LED chinh bang thanh ghi 0x10.
constexpr uint8_t I2C_SDA = 21;            // -> DR
constexpr uint8_t I2C_SCL = 22;            // -> CT
constexpr uint32_t I2C_HZ = 50000;         // de xuat 50 kHz; GY-33 cho phep 40-200 kHz

}  // namespace pins
