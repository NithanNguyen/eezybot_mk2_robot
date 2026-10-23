#include "color_console.h"
#include <stdlib.h>
#include <string.h>

namespace color_console {

static ColorClassifier* cc = nullptr;
static const char* fw = "?";
static uint32_t runPeriodMs = 0;
static uint32_t lastRunMs = 0;

void begin(ColorClassifier& c, const char* fwVersion) {
  cc = &c;
  fw = fwVersion;
}

static void printResult(const colormath::Result& r, float R, float G, float B, float C) {
  Serial.printf("CLS,%s,%s,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%d\n",
                colormath::labelName(r.label), colormath::labelName(r.nearest),
                r.dist, r.rn, r.gn, R, G, B, C, r.saturated ? 1 : 0);
}

static void printModel() {
  if (!cc->hasModel()) { Serial.println("MODEL,none"); return; }
  const colormath::Model& m = cc->model();
  Serial.printf("MODEL,%s,%u,%u,%u", cc->modelSource(), m.led, m.navg, m.settleMs);
  for (int k = 0; k < colormath::NUM_CLASSES; k++)
    Serial.printf(",%.7f,%.7f", m.cent[k][0], m.cent[k][1]);
  Serial.printf(",%.7f,%.3f,%.3f\n", m.dmax, m.cEmpty, m.cMargin);
}

colormath::Result classifyAndPrint() {
  colormath::Result r;
  memset(&r, 0, sizeof(r));
  r.label = colormath::UNKNOWN;
  r.nearest = colormath::UNKNOWN;
  if (!cc->sensorOk()) { Serial.println("ERR NO_SENSOR"); return r; }
  if (!cc->hasModel()) { Serial.println("ERR NO_MODEL"); return r; }
  RawMeas m;
  r = cc->classify(&m);
  if (!m.ok) { Serial.println("ERR READ_FAIL"); return r; }
  printResult(r, m.R, m.G, m.B, m.C);
  if (r.label == colormath::UNKNOWN) {
    Serial.printf("# ERR UNKNOWN_COLOR nearest=%s dist=%.4f dmax=%.4f sat=%d\n",
                  colormath::labelName(r.nearest), r.dist, cc->model().dmax, r.saturated ? 1 : 0);
  }
  return r;
}

bool handle(char* tok[], int n) {
  const char* cmd = tok[0];

  if (!strcmp(cmd, "PING")) {
    Serial.println("OK PONG");

  } else if (!strcmp(cmd, "INFO")) {
    int led = cc->sensorOk() ? cc->ledLevel() : -1;
    Serial.printf("INFO,fw=%s,sensor=%d,led=%d,model=%s\n", fw, cc->sensorOk() ? 1 : 0, led,
                  cc->hasModel() ? cc->modelSource() : "none");

  } else if (!strcmp(cmd, "LED")) {
    if (n < 2) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->sensorOk()) { Serial.println("ERR NO_SENSOR"); return true; }
    long lv = strtol(tok[1], nullptr, 10);
    if (lv < 0 || lv > GY33::LED_LEVEL_MAX) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->setLedLevel((uint8_t)lv)) { Serial.println("ERR I2C"); return true; }
    Serial.printf("OK LED,%ld\n", lv);

  } else if (!strcmp(cmd, "MEAS")) {
    if (n < 3) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->sensorOk()) { Serial.println("ERR NO_SENSOR"); return true; }
    int navg = atoi(tok[1]), settle = atoi(tok[2]);
    if (navg < 1 || navg > 50 || settle < 0 || settle > 5000) { Serial.println("ERR BAD_ARGS"); return true; }
    RawMeas m = cc->measure((uint8_t)navg, (uint16_t)settle);
    if (!m.ok) { Serial.println("ERR READ_FAIL"); return true; }
    Serial.printf("MEAS,%.3f,%.3f,%.3f,%.3f,%d,%d\n", m.R, m.G, m.B, m.C, m.saturated ? 1 : 0,
                  cc->ledLevel());

  } else if (!strcmp(cmd, "CLS")) {
    classifyAndPrint();

  } else if (!strcmp(cmd, "CLSRAW")) {
    if (n < 6) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->hasModel()) { Serial.println("ERR NO_MODEL"); return true; }
    float R = strtof(tok[1], nullptr), G = strtof(tok[2], nullptr);
    float B = strtof(tok[3], nullptr), C = strtof(tok[4], nullptr);
    printResult(cc->classifyRaw(R, G, B, C, atoi(tok[5]) != 0), R, G, B, C);

  } else if (!strcmp(cmd, "MODEL")) {
    if (n < 2) { Serial.println("ERR BAD_ARGS"); return true; }
    for (char* c = tok[1]; *c; c++) *c = toupper(*c);
    if (!strcmp(tok[1], "GET")) {
      printModel();
    } else if (!strcmp(tok[1], "CLEAR")) {
      cc->clearModel();
      Serial.println("OK MODEL,cleared");
    } else if (!strcmp(tok[1], "SET") && n >= 14) {
      colormath::Model m;
      memset(&m, 0, sizeof(m));
      m.magic = colormath::MODEL_MAGIC;
      m.led = (uint8_t)atoi(tok[2]);
      m.navg = (uint8_t)atoi(tok[3]);
      m.settleMs = (uint16_t)atoi(tok[4]);
      int t = 5;
      for (int k = 0; k < colormath::NUM_CLASSES; k++) {
        m.cent[k][0] = strtof(tok[t++], nullptr);
        m.cent[k][1] = strtof(tok[t++], nullptr);
      }
      m.dmax = strtof(tok[t++], nullptr);
      m.cEmpty = strtof(tok[t++], nullptr);
      m.cMargin = strtof(tok[t++], nullptr);
      if (cc->setModel(m, true, "nvs")) Serial.println("OK MODEL,nvs");
      else Serial.println("ERR MODEL_INVALID_OR_NVS_FAIL");
    } else {
      Serial.println("ERR BAD_ARGS");
    }

  } else if (!strcmp(cmd, "RUN")) {
    if (n < 2) { Serial.println("ERR BAD_ARGS"); return true; }
    long ms = atol(tok[1]);
    runPeriodMs = (ms > 0) ? (uint32_t)(ms < 200 ? 200 : ms) : 0;
    Serial.printf("OK RUN %lu\n", (unsigned long)runPeriodMs);

  } else {
    return false;
  }
  return true;
}

void update() {
  if (runPeriodMs && millis() - lastRunMs >= runPeriodMs) {
    lastRunMs = millis();
    classifyAndPrint();
  }
}

}  // namespace color_console
