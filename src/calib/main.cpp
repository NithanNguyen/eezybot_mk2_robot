// calib: hieu chuan servo (buoc 6). Dieu khien TUNG kenh bang do rong xung, doi xung tu tu (ramp) de do goc that.
// Quy trinh day du: docs/hieu_chuan_servo.md. Ket qua do -> tools/servo_cal/fit_servo_cal.py -> params.py.
//   pio run -e calib -t upload, Serial 115200, moi lenh mot dong:
//   ON <kenh> [us]     bat xung kenh tai us (mac dinh 1500). Nhay ngay toi us vi chua biet vi tri that.
//   OFF <kenh|ALL>     ngung xung (duty 0). STOP = OFF ALL.
//   SEL <kenh>         chon kenh cho US / + / -
//   US <us>            doi xung kenh dang chon toi us voi toc do RATE (us/s)
//   + [n] | - [n]      tang / giam n us (mac dinh 10)
//   RATE <us/s>        toc do doi xung, 10..500 (mac dinh 100)
//   LIM <kenh> <lo> <hi>   gioi han xung kenh, trong 500..2500
//   STATUS | ?         trang thai moi kenh;  HELP  in huong dan
// Kenh: 0|J1 = de (GPIO25), 1|J2 = vai (GPIO26), 2|J3 = khuyu (GPIO27), 3|G = kep SG90 (GPIO33).
#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "board_pins.h"
#include "ServoPwm.h"

namespace {

const uint8_t NCH = 4;
const uint8_t PINS[NCH] = {pins::SERVO_J1_BASE, pins::SERVO_J2_SHOULDER, pins::SERVO_J3_ELBOW, pins::SERVO_GRIPPER};
const char* const NAMES[NCH] = {"J1", "J2", "J3", "G"};
const char* const DESC[NCH] = {"de (MG996R)", "vai (MG996R)", "khuyu (MG996R)", "kep (SG90)"};
const uint16_t HARD_LO = 500, HARD_HI = 2500;     // gioi han cung cua lenh LIM
// Mac dinh hep (+-500 us quanh 1500): du de do do doc, tranh dua khop vao vung va cham khi chua biet chieu quay.
const uint16_t DEF_LO[NCH] = {1000, 1000, 1000, 900};
const uint16_t DEF_HI[NCH] = {2000, 2000, 2000, 2100};
const uint32_t TICK_MS = 20;

struct Channel {
  bool on;
  float cur;       // xung dang phat (us)
  float target;    // xung dich (us)
  uint16_t lo, hi;
  bool reported;   // da in "AT" cho dich hien tai
};

ServoPwm servos[NCH];
Channel ch[NCH];
int sel = -1;
float rate = 100.0f;   // us/s
uint32_t lastTick = 0;
char buf[96];
size_t len = 0;

int parseCh(const char* s) {
  if (!s) return -1;
  for (int i = 0; i < NCH; i++)
    if (!strcmp(s, NAMES[i])) return i;
  if (s[0] >= '0' && s[0] <= '3' && s[1] == 0) return s[0] - '0';
  return -1;
}

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void printStatus() {
  Serial.printf("STATUS,sel=%s,rate=%.0f\n", sel >= 0 ? NAMES[sel] : "-", (double)rate);
  for (int i = 0; i < NCH; i++)
    Serial.printf("  %s %-15s on=%d us=%.0f dich=%.0f lim=%u..%u\n", NAMES[i], DESC[i], ch[i].on ? 1 : 0,
                  (double)ch[i].cur, (double)ch[i].target, (unsigned)ch[i].lo, (unsigned)ch[i].hi);
}

void printHelp() {
  Serial.println("# calib: hieu chuan servo. Doc docs/hieu_chuan_servo.md truoc khi bat S1.");
  Serial.println("#   ON <kenh> [us] | OFF <kenh|ALL> | STOP | SEL <kenh> | US <us> | + [n] | - [n]");
  Serial.println("#   RATE <us/s> | LIM <kenh> <lo> <hi> | STATUS | HELP");
  Serial.println("#   kenh: J1 de, J2 vai, J3 khuyu, G kep (hoac 0..3). Moi kenh bat dau o trang thai OFF.");
}

void setTarget(int i, float us) {
  ch[i].target = clampf(us, ch[i].lo, ch[i].hi);
  ch[i].reported = false;
  if (us != ch[i].target) Serial.printf("# %s: %.0f ngoai lim %u..%u -> %.0f\n", NAMES[i], (double)us,
                                        (unsigned)ch[i].lo, (unsigned)ch[i].hi, (double)ch[i].target);
}

void handle(char* line) {
  char* tok[6];
  int n = 0;
  for (char* p = strtok(line, " \t"); p && n < 6; p = strtok(nullptr, " \t")) tok[n++] = p;
  if (!n) return;
  for (int k = 0; k < n; k++)
    for (char* c = tok[k]; *c; c++) *c = (char)toupper((unsigned char)*c);
  const char* c0 = tok[0];
  if (!strcmp(c0, "ON") && n >= 2) {
    const int i = parseCh(tok[1]);
    if (i < 0) { Serial.println("ERR KENH"); return; }
    const float us = clampf(n >= 3 ? (float)atof(tok[2]) : 1500.0f, ch[i].lo, ch[i].hi);
    ch[i].on = true;
    ch[i].cur = ch[i].target = us;
    ch[i].reported = true;
    servos[i].writeUs((uint32_t)lroundf(us));
    sel = i;
    Serial.printf("OK ON %s %.0f (nhay ngay, chua ramp)\n", NAMES[i], (double)us);
  } else if (!strcmp(c0, "OFF") && n >= 2 && !strcmp(tok[1], "ALL")) {
    for (int i = 0; i < NCH; i++) { servos[i].off(); ch[i].on = false; }
    Serial.println("OK OFF ALL");
  } else if (!strcmp(c0, "STOP")) {
    for (int i = 0; i < NCH; i++) { servos[i].off(); ch[i].on = false; }
    Serial.println("OK STOP");
  } else if (!strcmp(c0, "OFF") && n >= 2) {
    const int i = parseCh(tok[1]);
    if (i < 0) { Serial.println("ERR KENH"); return; }
    servos[i].off();
    ch[i].on = false;
    Serial.printf("OK OFF %s\n", NAMES[i]);
  } else if (!strcmp(c0, "SEL") && n >= 2) {
    const int i = parseCh(tok[1]);
    if (i < 0) { Serial.println("ERR KENH"); return; }
    sel = i;
    Serial.printf("OK SEL %s\n", NAMES[i]);
  } else if (!strcmp(c0, "US") || !strcmp(c0, "+") || !strcmp(c0, "-")) {
    if (sel < 0 || !ch[sel].on) { Serial.println("ERR CHUA_ON (dung ON <kenh> truoc)"); return; }
    float us;
    if (!strcmp(c0, "US")) {
      if (n < 2) { Serial.println("ERR THIEU_US"); return; }
      us = (float)atof(tok[1]);
    } else {
      const float d = n >= 2 ? (float)atof(tok[1]) : 10.0f;
      us = ch[sel].target + (c0[0] == '+' ? d : -d);
    }
    setTarget(sel, us);
    Serial.printf("OK %s -> %.0f\n", NAMES[sel], (double)ch[sel].target);
  } else if (!strcmp(c0, "RATE") && n >= 2) {
    rate = clampf((float)atof(tok[1]), 10.0f, 500.0f);
    Serial.printf("OK RATE %.0f\n", (double)rate);
  } else if (!strcmp(c0, "LIM") && n >= 4) {
    const int i = parseCh(tok[1]);
    const long lo = atol(tok[2]), hi = atol(tok[3]);
    if (i < 0 || lo < HARD_LO || hi > HARD_HI || lo >= hi) {
      Serial.printf("ERR LIM (kenh hop le, %u <= lo < hi <= %u)\n", (unsigned)HARD_LO, (unsigned)HARD_HI);
      return;
    }
    ch[i].lo = (uint16_t)lo;
    ch[i].hi = (uint16_t)hi;
    setTarget(i, ch[i].target);
    Serial.printf("OK LIM %s %ld..%ld\n", NAMES[i], lo, hi);
  } else if (!strcmp(c0, "STATUS") || !strcmp(c0, "?")) {
    printStatus();
  } else if (!strcmp(c0, "HELP")) {
    printHelp();
  } else {
    Serial.println("ERR LENH (HELP de xem)");
  }
}

void tick(float dt) {
  for (int i = 0; i < NCH; i++) {
    if (!ch[i].on) continue;
    const float step = rate * dt, d = ch[i].target - ch[i].cur;
    ch[i].cur += clampf(d, -step, step);
    servos[i].writeUs((uint32_t)lroundf(ch[i].cur));
    if (!ch[i].reported && fabsf(ch[i].target - ch[i].cur) < 0.5f) {
      ch[i].reported = true;
      Serial.printf("AT %s %.0f\n", NAMES[i], (double)ch[i].cur);
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  for (int i = 0; i < NCH; i++) {
    servos[i].attach(PINS[i], (uint8_t)i, HARD_LO, HARD_HI);
    servos[i].off();
    ch[i].on = false;
    ch[i].cur = ch[i].target = 1500.0f;
    ch[i].lo = DEF_LO[i];
    ch[i].hi = DEF_HI[i];
    ch[i].reported = true;
  }
  printHelp();
  Serial.println("# READY calib");
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
