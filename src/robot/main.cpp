// Firmware chinh cua tay may EEZYbotARM Mk2 - phan loai nap chai theo mau.
//   pio run -e robot -t upload
//
// Serial 115200, moi lenh mot dong:
//   - Lenh cam bien mau (PING, INFO, LED, MEAS, CLS, MODEL, ...): xem color_console.h.
//     tools/colorcal/colorcal.py dung cac lenh nay de hieu chuan -> khong can nap firmware khac.
//   - AUTO 1 | AUTO 0   bat/tat chu trinh phan loai tu dong (mac dinh tat)
//   - STATUS            trang thai + so nap da phan loai
#include <Arduino.h>
#include <string.h>
#include "board_pins.h"
#include "ColorClassifier.h"
#include "color_console.h"
#include "arm.h"
#include "sorter.h"

#if __has_include("color_model.h")
#include "color_model.h"   // sinh boi: python3 tools/colorcal/colorcal.py train
#endif

static const char* FW_VERSION = "robot-0.2";

static ColorClassifier color(pins::I2C_SDA, pins::I2C_SCL, pins::I2C_HZ);
static Arm arm;
static Sorter sorter(color, arm);

static char lineBuf[256];
static size_t lineLen = 0;

static void loadFallbackModel() {
#ifdef COLOR_MODEL_DEFINED
  // Model bien dich san chi dung khi NVS chua co model (vd. sau khi Erase Flash).
  if (color.hasModel()) return;
  colormath::Model m;
  memset(&m, 0, sizeof(m));
  m.magic = colormath::MODEL_MAGIC;
  m.led = CM_LED; m.navg = CM_NAVG; m.settleMs = CM_SETTLE_MS;
  const float cent[3][2] = CM_CENT;
  memcpy(m.cent, cent, sizeof(cent));
  m.dmax = CM_DMAX; m.cEmpty = CM_C_EMPTY; m.cMargin = CM_C_MARGIN;
  color.setModel(m, false, "header");
#endif
}

static int tokenize(char* s, char* tok[], int maxTok) {
  int n = 0;
  for (char* p = strtok(s, " \t,"); p && n < maxTok; p = strtok(nullptr, " \t,")) tok[n++] = p;
  return n;
}

static void handleLine(char* line) {
  char* tok[20];
  int n = tokenize(line, tok, 20);
  if (n == 0) return;
  for (char* c = tok[0]; *c; c++) *c = toupper(*c);

  if (color_console::handle(tok, n)) return;

  if (!strcmp(tok[0], "AUTO") && n >= 2) {
    sorter.setAuto(atoi(tok[1]) != 0);
  } else if (!strcmp(tok[0], "STATUS")) {
    Serial.printf("STATUS,fw=%s,auto=%d,sensor=%d,model=%s,green=%lu,red=%lu,white=%lu,reject=%lu\n",
                  FW_VERSION, sorter.isAuto() ? 1 : 0, color.sensorOk() ? 1 : 0,
                  color.hasModel() ? color.modelSource() : "none",
                  (unsigned long)sorter.count(Bin::GREEN), (unsigned long)sorter.count(Bin::RED),
                  (unsigned long)sorter.count(Bin::WHITE), (unsigned long)sorter.count(Bin::REJECT));
  } else {
    Serial.println("ERR UNKNOWN_CMD");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  bool ok = color.begin();
  loadFallbackModel();
  color_console::begin(color, FW_VERSION);
  arm.begin();

  Serial.printf("# READY %s sensor=%d model=%s\n", FW_VERSION, ok ? 1 : 0,
                color.hasModel() ? color.modelSource() : "none");
  if (!ok) Serial.println("# ERR GY-33 khong phan hoi tai 0x5A - kiem tra day DR/CT/VCC/GND va jumper S0-G");
  if (!color.hasModel()) Serial.println("# chua co model mau - chay: python3 tools/colorcal/colorcal.py wizard");
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    }
  }
  color_console::update();
  sorter.update();
}
