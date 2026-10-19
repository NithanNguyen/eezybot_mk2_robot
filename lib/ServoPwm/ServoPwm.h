// ServoPwm.h - phat xung servo 50 Hz bang bo LEDC cua ESP32.
// Tach tu servo_center/src/main.cpp; tuong thich Arduino-ESP32 core 2.x
// (ledcSetup/ledcAttachPin) va 3.x (ledcAttach, ledcWrite theo chan).
#pragma once
#include <Arduino.h>

class ServoPwm {
 public:
  static const uint32_t PWM_HZ = 50;                      // chu ky 20 ms
  static const uint8_t PWM_BITS = 16;                     // do phan giai duty
  static const uint32_t PERIOD_US = 1000000UL / PWM_HZ;   // 20000 us

  // channel: kenh LEDC (chi dung o core 2.x), moi servo mot kenh khac nhau.
  // minUs/maxUs: gioi han phan mem; writeUs() kep gia tri vao khoang nay.
  void attach(uint8_t pin, uint8_t channel, uint16_t minUs = 500, uint16_t maxUs = 2500);
  void writeUs(uint32_t us);
  uint32_t lastUs() const { return _lastUs; }
  uint8_t pin() const { return _pin; }

  static uint32_t usToDuty(uint32_t us) {
    return (uint32_t)(((uint64_t)us * ((1UL << PWM_BITS) - 1)) / PERIOD_US);
  }

 private:
  uint8_t _pin = 255, _channel = 0;
  uint16_t _minUs = 500, _maxUs = 2500;
  uint32_t _lastUs = 0;
};
