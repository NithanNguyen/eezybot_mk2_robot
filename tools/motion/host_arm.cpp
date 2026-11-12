// Chay src/robot/arm.cpp (task 20 ms, hang doi, lenh "ARM ...") tren PC voi stub FreeRTOS trong tools/motion/stub.
// Task chuyen dong chay tren mot thread rieng; thoi gian mo phong tang 20 ms moi nhip (khong cho thuc 20 ms).
// Dung boi tools/motion/test_motion_parity.py de so trace cua lop Arduino voi mo phong. KHONG dung cho ESP32.
// stdin: moi dong mot lenh Serial (vd. "ARM TRACE 1", "ARM SORT RED"); stdout: dung nhu Serial cua ESP32.
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstring>
#include <deque>
#include <string>
#include <thread>
#include <vector>
#include "Arduino.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "../../src/robot/arm.h"

HostSerial Serial;

// ------------------------------------------------------------------ stub thoi gian
static std::atomic<int64_t> g_simUs(0);
static int64_t realNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
static std::atomic<int64_t> g_realTickStartNs(realNs());
int64_t esp_timer_get_time() { return g_simUs.load() + (realNs() - g_realTickStartNs.load()) / 1000; }
TickType_t xTaskGetTickCount() { return (TickType_t)(g_simUs.load() / 1000); }
void vTaskDelayUntil(TickType_t* last, TickType_t period) {
  *last += period;
  std::this_thread::sleep_for(std::chrono::microseconds(300));  // de loop() kip in
  g_realTickStartNs = realNs();
  g_simUs = (int64_t)*last * 1000;
}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char*, uint32_t, void* arg, UBaseType_t, TaskHandle_t* h,
                                   BaseType_t) {
  std::thread(fn, arg).detach();
  if (h) *h = nullptr;
  return pdTRUE;
}

// ------------------------------------------------------------------ stub hang doi
struct HostQueue {
  std::mutex m;
  std::deque<std::vector<uint8_t>> q;
  size_t len, size;
};
QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t itemSize) {
  HostQueue* q = new HostQueue();
  q->len = len;
  q->size = itemSize;
  return q;
}
BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t) {
  std::lock_guard<std::mutex> l(q->m);
  if (q->q.size() >= q->len) return pdFALSE;
  const uint8_t* p = (const uint8_t*)item;
  q->q.emplace_back(p, p + q->size);
  return pdTRUE;
}
static BaseType_t take(QueueHandle_t q, void* item, bool pop) {
  std::lock_guard<std::mutex> l(q->m);
  if (q->q.empty()) return pdFALSE;
  std::memcpy(item, q->q.front().data(), q->size);
  if (pop) q->q.pop_front();
  return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void* item, TickType_t) { return take(q, item, true); }
BaseType_t xQueuePeek(QueueHandle_t q, void* item, TickType_t) { return take(q, item, false); }
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) {
  std::lock_guard<std::mutex> l(q->m);
  return (UBaseType_t)q->q.size();
}

// ------------------------------------------------------------------ "loop()"
static Arm arm;

static void settle() {
  do {
    arm.update();
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  } while (arm.busy());
  for (int i = 0; i < 50; i++) {
    arm.update();
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
}

int main() {
  arm.begin();
  settle();
  std::string line;
  char buf[256];
  while (std::fgets(buf, sizeof buf, stdin)) {
    char* tok[20];
    int n = 0;
    for (char* p = std::strtok(buf, " \t\r\n,"); p && n < 20; p = std::strtok(nullptr, " \t\r\n,")) tok[n++] = p;
    if (!n) continue;
    for (char* c = tok[0]; *c; c++) *c = (char)std::toupper((unsigned char)*c);
    if (!arm.handleCommand(tok, n)) Serial.println("ERR UNKNOWN_CMD");
    settle();
  }
  return 0;
}
