// Stub Arduino toi thieu de chay src/robot/arm.cpp tren PC (tools/motion/host_arm.cpp). KHONG dung cho ESP32.
#pragma once
#include <cstdint>
#include <cstdio>
#include <mutex>
struct HostSerial {
  std::mutex m;
  template <class... A>
  int printf(const char* f, A... a) {
    std::lock_guard<std::mutex> l(m);
    const int r = std::printf(f, a...);
    std::fflush(stdout);
    return r;
  }
  void println(const char* s) {
    std::lock_guard<std::mutex> l(m);
    std::puts(s);
    std::fflush(stdout);
  }
};
extern HostSerial Serial;
// Nhu Arduino.h that: cac macro cua Arduino-ESP32 + newlib <ctype.h> co hieu luc voi moi thu include sau.
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../../host_compat/arduino_macro_trap.h"
