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
