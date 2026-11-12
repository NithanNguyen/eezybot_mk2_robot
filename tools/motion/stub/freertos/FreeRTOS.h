// Stub FreeRTOS toi thieu (thread + mutex cua C++) de chay arm.cpp tren PC. Chi du cho kiem thu logic.
#pragma once
#include <cstdint>
#include <mutex>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
struct portMUX_TYPE {
  std::recursive_mutex m;
};
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(x) (x)->m.lock()
#define portEXIT_CRITICAL(x) (x)->m.unlock()
