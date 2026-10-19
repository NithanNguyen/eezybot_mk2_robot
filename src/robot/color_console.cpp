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
  if (!cc->hasModel()) {
    Serial.println("MODEL,none");
    return;
  }
  const colormath::Model& m = cc->model();
  Serial.printf("MODEL,%s,%u,%u,%u,%u", cc->modelSource(), m.atime, m.gain, m.navg, m.nflush);
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
    Serial.printf("INFO,fw=%s,sensor=%d,atime=%u,gain=%u,sat=%lu,it_ms=%.1f,model=%s\n",
                  fw, cc->sensorOk() ? 1 : 0, cc->atime(), cc->gain(),
                  (unsigned long)cc->saturation(), colormath::integrationMs(cc->atime()),
                  cc->hasModel() ? cc->modelSource() : "none");

  } else if (!strcmp(cmd, "LED")) {
    if (n < 2) { Serial.println("ERR BAD_ARGS"); return true; }
    bool on = atoi(tok[1]) != 0;
    cc->setLed(on);
    Serial.printf("OK LED %d\n", on ? 1 : 0);

  } else if (!strcmp(cmd, "CFG")) {
    if (n < 3) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->sensorOk()) { Serial.println("ERR NO_SENSOR"); return true; }
    long at = strtol(tok[1], nullptr, 0);   // nhan ca "235" lan "0xEB"
    long g = strtol(tok[2], nullptr, 0);
    if (at < 0 || at > 255 || g < 0 || g > 3) { Serial.println("ERR BAD_ARGS"); return true; }
    cc->configure((uint8_t)at, (uint8_t)g);
    Serial.printf("OK CFG,%u,%u,%lu,%.1f\n", cc->atime(), cc->gain(),
                  (unsigned long)cc->saturation(), colormath::integrationMs(cc->atime()));

  } else if (!strcmp(cmd, "MEAS")) {
    if (n < 3) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->sensorOk()) { Serial.println("ERR NO_SENSOR"); return true; }
    int navg = atoi(tok[1]), nflush = atoi(tok[2]);
    if (navg < 1 || navg > 50 || nflush < 0 || nflush > 10) { Serial.println("ERR BAD_ARGS"); return true; }
    RawMeas m = cc->measure((uint8_t)navg, (uint8_t)nflush);
    if (!m.ok) { Serial.println("ERR READ_FAIL"); return true; }
    Serial.printf("MEAS,%.3f,%.3f,%.3f,%.3f,%d,%u,%u\n", m.R, m.G, m.B, m.C,
                  m.saturated ? 1 : 0, cc->atime(), cc->gain());

  } else if (!strcmp(cmd, "CLS")) {
    classifyAndPrint();

  } else if (!strcmp(cmd, "CLSRAW")) {
    if (n < 6) { Serial.println("ERR BAD_ARGS"); return true; }
    if (!cc->hasModel()) { Serial.println("ERR NO_MODEL"); return true; }
    float R = strtof(tok[1], nullptr), G = strtof(tok[2], nullptr);
    float B = strtof(tok[3], nullptr), C = strtof(tok[4], nullptr);
    bool sat = atoi(tok[5]) != 0;
    printResult(cc->classifyRaw(R, G, B, C, sat), R, G, B, C);

  } else if (!strcmp(cmd, "MODEL")) {
    if (n < 2) { Serial.println("ERR BAD_ARGS"); return true; }
    for (char* c = tok[1]; *c; c++) *c = toupper(*c);
    if (!strcmp(tok[1], "GET")) {
      printModel();
    } else if (!strcmp(tok[1], "CLEAR")) {
      cc->clearModel();
      Serial.println("OK MODEL,cleared");
    } else if (!strcmp(tok[1], "SET") && n >= 15) {
      colormath::Model m;
      memset(&m, 0, sizeof(m));
      m.magic = colormath::MODEL_MAGIC;
      m.atime = (uint8_t)strtol(tok[2], nullptr, 0);
      m.gain = (uint8_t)strtol(tok[3], nullptr, 0);
      m.navg = (uint8_t)atoi(tok[4]);
      m.nflush = (uint8_t)atoi(tok[5]);
      int t = 6;
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
    runPeriodMs = (ms > 0) ? (uint32_t)(ms < 100 ? 100 : ms) : 0;
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
