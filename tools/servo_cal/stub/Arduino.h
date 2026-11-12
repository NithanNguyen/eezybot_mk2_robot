// Stub Arduino cho tools/servo_cal/host_calib_test.cpp: Serial co hang doi nhap, millis() dieu khien duoc,
// LEDC ghi lai duty (API core 2.x). KHONG dung cho ESP32.
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
struct HostSerial {
  std::string in, out;
  void begin(unsigned long) {}
  int available() { return (int)in.size(); }
  int read() {
    if (in.empty()) return -1;
    const int c = (unsigned char)in[0];
    in.erase(0, 1);
    return c;
  }
  template <class... A>
  int printf(const char* f, A... a) {
    char b[256];
    const int r = std::snprintf(b, sizeof b, f, a...);
    out += b;
    return r;
  }
  void println(const char* s) { out += s; out += "\n"; }
};
extern HostSerial Serial;
// Bay macro Arduino-ESP32 (nhu Arduino.h that). API LEDC duoc firmware GOI co chu y (ServoPwm.cpp) nen bo bay
// cho dung cac ten nay roi khai bao stub ben duoi.
#include "../../host_compat/arduino_macro_trap.h"
#undef ledcSetup
#undef ledcAttachPin
#undef ledcAttach
#undef ledcWrite
extern uint32_t g_ms;
extern uint32_t g_duty[16];
inline uint32_t millis() { return g_ms; }
inline void delay(uint32_t ms) { g_ms += ms; }
// Ca hai API: core 2.x (ledcSetup/ledcAttachPin, ledcWrite theo kenh) va 3.x (ledcAttach, ledcWrite theo chan).
// g_duty danh so theo kenh 0..3; g_pinCh doi chan -> kenh.
extern int g_pinCh[64];
inline void ledcSetup(uint8_t, uint32_t, uint8_t) {}
inline void ledcAttachPin(uint8_t pin, uint8_t ch) { g_pinCh[pin] = ch; }
inline bool ledcAttach(uint8_t pin, uint32_t, uint8_t) {
  static int next = 0;
  if (g_pinCh[pin] < 0) g_pinCh[pin] = next++;
  return true;
}
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
inline bool ledcWrite(uint8_t pin, uint32_t duty) { g_duty[g_pinCh[pin]] = duty; return true; }
#else
inline void ledcWrite(uint8_t ch, uint32_t duty) { g_duty[ch] = duty; }
#endif
