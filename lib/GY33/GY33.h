// GY33.h - doc module mau GY-33 (TCS34725 + MCU) o che do MCU_IIC.
//
// Theo tai lieu hang (GY-33 使用说明 V1.0, tr.5-6) va ma mau arduino_iic.ino:
//  - Pad S0 noi GND -> MCU_IIC; CT = SCL, DR = SDA; dia chi 7-bit 0x5A.
//  - Xung SCL 40-200 kHz; du lieu cap nhat ~10 Hz.
//  - 0x00-0x07: R, G, B, C tho 16 bit, byte cao truoc ("chua qua xu ly").
//  - 0x10 (doc/ghi): bit7-4 = muc LED 0-10 (so nho = sang hon, mac dinh 3,
//    khong luu khi mat dien); bit0 = 1 -> hieu chuan trang (khong dung o day,
//    vi phan loai dung RGBC tho).
// Thoi gian cau hinh tich phan/do loi cua TCS34725 do MCU tren module quyet dinh,
// tai lieu khong cho phep doi va khong neu gia tri.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class GY33 {
 public:
  static const uint8_t DEFAULT_ADDR = 0x5A;
  static const uint8_t LED_LEVEL_MAX = 10;
  static const uint8_t LED_LEVEL_DEFAULT = 3;
  static const uint32_t UPDATE_PERIOD_MS = 100;   // ~10 Hz

  explicit GY33(uint8_t addr = DEFAULT_ADDR) : _addr(addr) {}

  // Wire phai duoc khoi dong truoc (Wire.begin(sda, scl, hz)).
  bool begin(TwoWire& wire = Wire);
  bool present();                              // ACK tai dia chi
  bool readRaw(uint16_t rgbc[4]);              // R, G, B, C
  int readLedLevel();                          // 0..10, -1 neu loi
  bool writeLedLevel(uint8_t level);           // ghi 0x10 = level << 4 (bit0 = 0)

 private:
  bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
  TwoWire* _wire = nullptr;
  uint8_t _addr;
};
