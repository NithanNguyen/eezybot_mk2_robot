// Kiem thu logic firmware hieu chuan (src/calib/main.cpp) tren PC: chay setup()/loop() that voi stub Serial,
// millis() va LEDC. Kiem: bat dau OFF, ON nhay toi xung, ramp dung RATE, kep vao LIM, OFF/STOP -> duty 0,
// lenh sai bi tu choi. Chay: python3 tools/servo_cal/test_calib_host.py
#include <cstdio>
#include "Arduino.h"
HostSerial Serial;
uint32_t g_ms = 0;
uint32_t g_duty[16] = {0};
int g_pinCh[64] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                   -1, -1, -1, -1};
#include "../../lib/ServoPwm/ServoPwm.cpp"
#include "../../src/calib/main.cpp"

static int fails = 0;
static void check(bool ok, const char* what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}
static double us(int chn) { return g_duty[chn] * 20000.0 / 65535.0; }   // duty 16 bit, chu ky 20 ms
static void cmd(const char* c) { Serial.in += c; Serial.in += "\n"; loop(); }
static void run(uint32_t ms) { for (uint32_t i = 0; i < ms; i++) { g_ms++; loop(); } }
static bool outHas(const char* s) { const bool r = Serial.out.find(s) != std::string::npos; return r; }

int main() {
  setup();
  check(g_duty[0] == 0 && g_duty[1] == 0 && g_duty[2] == 0 && g_duty[3] == 0, "khoi dong: ca 4 kenh khong phat xung");
  cmd("US 1600");
  check(outHas("ERR CHUA_ON"), "US khi chua ON bi tu choi");
  cmd("on j2");
  check(fabs(us(1) - 1500) < 0.5 && g_duty[0] == 0 && g_duty[2] == 0 && g_duty[3] == 0, "ON J2: chi J2 phat 1500 us");
  cmd("US 1700");
  run(1000);                      // 1 s voi RATE 100 us/s -> ~1600
  check(fabs(us(1) - 1600) < 3.0, "ramp 100 us/s: sau 1 s o ~1600 us");
  run(1100);
  check(fabs(us(1) - 1700) < 0.5 && outHas("AT J2 1700"), "toi dich 1700 us va in AT");
  cmd("US 2400");
  run(5000);
  check(fabs(us(1) - 2000) < 0.5 && outHas("ngoai lim"), "US vuot LIM mac dinh -> kep 2000 us");
  cmd("RATE 500"); cmd("- 300"); run(700);
  check(fabs(us(1) - 1700) < 0.5, "RATE 500 va '- 300' -> 1700 us");
  cmd("LIM J2 400 2600");
  check(outHas("ERR LIM"), "LIM ngoai 500..2500 bi tu choi");
  cmd("LIM 1 1200 1650"); run(500);
  check(fabs(us(1) - 1650) < 0.5, "thu hep LIM keo dich ve 1650 us");
  cmd("ON G 1000");
  check(fabs(us(3) - 1000) < 0.5 && fabs(us(1) - 1650) < 0.5, "ON G 1000: kep phat 1000, J2 giu nguyen");
  cmd("OFF J2");
  check(g_duty[1] == 0 && us(3) > 999, "OFF J2: J2 duty 0, kep van phat");
  cmd("SEL J2"); cmd("+ 10");
  check(outHas("ERR CHUA_ON"), "+ tren kenh da OFF bi tu choi");
  cmd("STOP");
  check(g_duty[0] == 0 && g_duty[1] == 0 && g_duty[2] == 0 && g_duty[3] == 0, "STOP: ca 4 kenh duty 0");
  cmd("ON X");
  check(outHas("ERR KENH"), "kenh khong hop le bi tu choi");
  std::printf("%s (%d loi)\n", fails ? "CO LOI" : "TAT CA DAT", fails);
  return fails ? 1 : 0;
}
