#include "ColorClassifier.h"
#include <Preferences.h>
#include <string.h>

static const char* NVS_NS = "colorcls";
static const char* NVS_KEY = "model";
// Tai lieu khong neu thoi gian khoi dong cua GY-33; ma mau hang cho 1 ms.
// 100 ms la uoc luong an toan (theo mk2_connection_diagram, muc 5).
static const uint32_t GY33_BOOT_MS = 100;

ColorClassifier::ColorClassifier(uint8_t sdaPin, uint8_t sclPin, uint32_t i2cHz)
    : _sda(sdaPin), _scl(sclPin), _hz(i2cHz) {
  memset(&_model, 0, sizeof(_model));
}

bool ColorClassifier::begin() {
  Wire.begin(_sda, _scl, _hz);
  delay(GY33_BOOT_MS);
  _sensorOk = _gy.begin(Wire);
  loadModelFromNvs();
  if (_sensorOk && hasModel()) ensureModelLed();
  return _sensorOk;
}

// Muc LED khong duoc GY-33 luu khi mat dien -> dat lai theo model neu khac.
void ColorClassifier::ensureModelLed() {
  int cur = _gy.readLedLevel();
  if (cur == _model.led) return;
  _gy.writeLedLevel(_model.led);
  delay(3 * GY33::UPDATE_PERIOD_MS);
  if (cur >= 0) Serial.printf("# GY-33 LED %d -> %u (theo model)\n", cur, _model.led);
}

RawMeas ColorClassifier::measure(uint8_t navg, uint16_t settleMs) {
  RawMeas m = {0, 0, 0, 0, false, false};
  if (!_sensorOk || navg == 0) return m;
  delay(settleMs);   // cho du lieu cu (truoc khi dat nap) duoc thay the

  uint32_t s[4] = {0, 0, 0, 0};
  for (uint8_t i = 0; i < navg; i++) {
    if (i) delay(READ_INTERVAL_MS);
    uint16_t v[4];
    if (!_gy.readRaw(v)) return m;   // ok = false
    for (int k = 0; k < 4; k++) {
      s[k] += v[k];
      if (v[k] >= colormath::RAW_MAX) m.saturated = true;
    }
  }
  m.R = (float)s[0] / navg;
  m.G = (float)s[1] / navg;
  m.B = (float)s[2] / navg;
  m.C = (float)s[3] / navg;
  m.ok = true;
  return m;
}

colormath::Result ColorClassifier::classify(RawMeas* rawOut) {
  colormath::Result res;
  memset(&res, 0, sizeof(res));
  res.label = colormath::UNKNOWN;
  res.nearest = colormath::UNKNOWN;
  if (!hasModel() || !_sensorOk) return res;

  ensureModelLed();
  RawMeas m = measure(_model.navg, _model.settleMs);
  if (rawOut) *rawOut = m;
  if (!m.ok) return res;
  return colormath::classify(_model, m.R, m.G, m.B, m.C, m.saturated);
}

bool ColorClassifier::setModel(const colormath::Model& m, bool persist, const char* src) {
  if (!colormath::modelValid(m)) return false;
  _model = m;
  _modelSrc = src;
  if (_sensorOk) ensureModelLed();
  if (persist) {
    Preferences p;
    if (!p.begin(NVS_NS, false)) return false;
    size_t n = p.putBytes(NVS_KEY, &_model, sizeof(_model));
    p.end();
    if (n != sizeof(_model)) return false;
  }
  return true;
}

void ColorClassifier::clearModel() {
  Preferences p;
  if (p.begin(NVS_NS, false)) {
    p.remove(NVS_KEY);
    p.end();
  }
  memset(&_model, 0, sizeof(_model));
  _modelSrc = "none";
}

bool ColorClassifier::loadModelFromNvs() {
  Preferences p;
  if (!p.begin(NVS_NS, true)) return false;
  colormath::Model m;
  size_t n = 0;
  if (p.getBytesLength(NVS_KEY) == sizeof(m)) n = p.getBytes(NVS_KEY, &m, sizeof(m));
  p.end();
  if (n != sizeof(m) || !colormath::modelValid(m)) return false;   // model cu (CM01) bi bo qua
  _model = m;
  _modelSrc = "nvs";
  return true;
}
