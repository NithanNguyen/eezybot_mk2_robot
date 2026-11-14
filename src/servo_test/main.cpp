// servo_test: kiem tra mot servo (mac dinh SG90 kep o GPIO33) va duong tin hieu toi no.
// Dung khi mot kenh khong dap ung o buoc B3 cua docs/hieu_chuan_servo.md.
// Quy trinh chan doan: docs/kiem_tra_sg90.md.
//   pio run -e servo_test -t upload, Serial 115200, moi lenh mot dong:
//   US <us>           phat xung us (kep trong 900..2100) tren chan dang chon
//   SWEEP [lo hi]     quet qua lai lo..hi (mac dinh 1300..1700) voi toc do RATE
//   RATE <us/s>       toc do quet, 10..500 (mac dinh 200)
//   OFF               ngung xung (duty 0)
//   PIN <25|26|27|33> doi chan phat xung (de doi cheo servo/chan)
//   MEAS              do do rong xung tren GPIO34 (noi day tu diem can kiem tra vao GPIO34)
//   ? | STATUS        trang thai
#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "board_pins.h"
#include "ServoPwm.h"

namespace {

const uint8_t ALLOWED_PINS[] = {pins::SERVO_J1_BASE, pins::SERVO_J2_SHOULDER, pins::SERVO_J3_ELBOW,
                                pins::SERVO_GRIPPER};
const uint8_t MEAS_PIN = 34;          // chi vao, khong dung o noi khac trong robot
const uint8_t LEDC_CH = 0;            // chi dung 1 kenh LEDC (core 2.x)
const uint16_t US_LO = 900, US_HI = 2100;
const uint32_t TICK_MS = 20;

ServoPwm servo;
uint8_t outPin = pins::SERVO_GRIPPER;
bool on = false;
bool sweeping = false;
float cur = 1500.0f, target = 1500.0f;
float sweepLo = 1300.0f, sweepHi = 1700.0f;
float rate = 200.0f;
uint32_t lastTick = 0;
char buf[64];
size_t len = 0;

float clampUs(float v) { return v < US_LO ? US_LO : (v > US_HI ? US_HI : v); }

bool pinAllowed(long p) {
  for (uint8_t a : ALLOWED_PINS)
    if (a == p) return true;
  return false;
}

void detachOut() {
  servo.off();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcDetach(outPin);
#else
  ledcDetachPin(outPin);
#endif
  pinMode(outPin, INPUT);
}

void status() {
  Serial.printf("STATUS pin=GPIO%u on=%d sweep=%d us=%.0f target=%.0f sweep=%.0f..%.0f rate=%.0f meas=GPIO%u\n",
                (unsigned)outPin, on ? 1 : 0, sweeping ? 1 : 0, (double)cur, (double)target,
                (double)sweepLo, (double)sweepHi, (double)rate, (unsigned)MEAS_PIN);
}

// Do 10 xung muc cao tren GPIO34. pulseIn tra 0 neu khong thay xung trong 25 ms.
void measure() {
  uint32_t n = 0, sum = 0, mn = 0xFFFFFFFFu, mx = 0;
  for (int i = 0; i < 10; i++) {
    const uint32_t w = pulseIn(MEAS_PIN, HIGH, 25000);
    if (w == 0) continue;
    n++;
    sum += w;
    if (w < mn) mn = w;
    if (w > mx) mx = w;
  }
  if (n == 0) {
    Serial.printf("MEAS GPIO%u: KHONG CO XUNG (muc tinh = %d)\n", (unsigned)MEAS_PIN, digitalRead(MEAS_PIN));
    return;
  }
  Serial.printf("MEAS GPIO%u: %lu/10 xung, TB=%lu us, min=%lu, max=%lu (lenh hien tai %.0f us)\n",
                (unsigned)MEAS_PIN, (unsigned long)n, (unsigned long)(sum / n), (unsigned long)mn,
                (unsigned long)mx, (double)(on ? cur : 0.0f));
}

void handle(char* line) {
  char* tok[4];
  int n = 0;
  for (char* p = strtok(line, " \t"); p && n < 4; p = strtok(nullptr, " \t")) tok[n++] = p;
  if (!n) return;
  for (char* c = tok[0]; *c; c++) *c = (char)toupper((unsigned char)*c);
  const char* c0 = tok[0];
  if (!strcmp(c0, "US") && n >= 2) {
    target = clampUs((float)atof(tok[1]));
    if (!on) cur = target;   // lan dau: nhay ngay toi xung dat
    on = true;
    sweeping = false;
    Serial.printf("OK US %.0f\n", (double)target);
  } else if (!strcmp(c0, "SWEEP")) {
    if (n >= 3) {
      sweepLo = clampUs((float)atof(tok[1]));
      sweepHi = clampUs((float)atof(tok[2]));
      if (sweepLo > sweepHi) { const float t = sweepLo; sweepLo = sweepHi; sweepHi = t; }
    }
    if (!on) cur = 1500.0f;
    on = true;
    sweeping = true;
    target = sweepHi;
    Serial.printf("OK SWEEP %.0f..%.0f @ %.0f us/s\n", (double)sweepLo, (double)sweepHi, (double)rate);
  } else if (!strcmp(c0, "RATE") && n >= 2) {
    float r = (float)atof(tok[1]);
    rate = r < 10 ? 10 : (r > 500 ? 500 : r);
    Serial.printf("OK RATE %.0f\n", (double)rate);
  } else if (!strcmp(c0, "OFF") || !strcmp(c0, "STOP")) {
    servo.off();
    on = false;
    sweeping = false;
    Serial.println("OK OFF");
  } else if (!strcmp(c0, "PIN") && n >= 2) {
    const long p = atol(tok[1]);
    if (!pinAllowed(p)) { Serial.println("ERR PIN (chi 25, 26, 27, 33)"); return; }
    detachOut();
    outPin = (uint8_t)p;
    servo.attach(outPin, LEDC_CH, US_LO, US_HI);
    servo.off();
    on = false;
    sweeping = false;
    Serial.printf("OK PIN GPIO%u (dang OFF, go US hoac SWEEP)\n", (unsigned)outPin);
  } else if (!strcmp(c0, "MEAS")) {
    measure();
  } else if (!strcmp(c0, "?") || !strcmp(c0, "STATUS")) {
    status();
  } else {
    Serial.println("ERR LENH: US <us> | SWEEP [lo hi] | RATE <us/s> | OFF | PIN <25|26|27|33> | MEAS | ?");
  }
}

void tick(float dt) {
  if (!on) return;
  const float step = rate * dt, d = target - cur;
  if (d > step) cur += step;
  else if (d < -step) cur -= step;
  else cur = target;   // da toi dich
  if (sweeping && cur == target) target = (target == sweepHi) ? sweepLo : sweepHi;
  servo.writeUs((uint32_t)(cur + 0.5f));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(MEAS_PIN, INPUT);
  servo.attach(outPin, LEDC_CH, US_LO, US_HI);
  servo.off();
  Serial.println("# servo_test: US <us> | SWEEP [lo hi] | RATE <us/s> | OFF | PIN <25|26|27|33> | MEAS | ?");
  Serial.printf("# chan phat mac dinh GPIO%u (SG90 kep), do xung tren GPIO%u. Dang OFF.\n",
                (unsigned)outPin, (unsigned)MEAS_PIN);
  Serial.println("# READY servo_test");
  lastTick = millis();
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      handle(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
  const uint32_t now = millis();
  if (now - lastTick >= TICK_MS) {
    tick((float)(now - lastTick) / 1000.0f);
    lastTick = now;
  }
}
