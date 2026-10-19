#include "ColorClassifier.h"
#include <Preferences.h>
#include <string.h>

// Thoi gian cho sau khi bat LED truoc khi doc (ms). Phan lon do on dinh
// da duoc xu ly bang cac lan doc bo di (nflush); gia tri nho nay chi de an toan.
#ifndef COLOR_LED_SETTLE_MS
#define COLOR_LED_SETTLE_MS 10
#endif

static const char* NVS_NS = "colorcls";
static const char* NVS_KEY = "model";

ColorClassifier::ColorClassifier(uint8_t ledPin, uint8_t sdaPin, uint8_t sclPin)
    : _tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
      _ledPin(ledPin), _sda(sdaPin), _scl(sclPin),
      _atime(TCS34725_INTEGRATIONTIME_50MS), _gain(1),
      _sensorOk(false), _modelSrc("none") {
  memset(&_model, 0, sizeof(_model));
}

bool ColorClassifier::begin() {
  pinMode(_ledPin, OUTPUT);
  digitalWrite(_ledPin, LOW);
  Wire.begin(_sda, _scl);
  _sensorOk = _tcs.begin(TCS34725_ADDRESS, &Wire);
  if (loadModelFromNvs()) {
    configure(_model.atime, _model.gain);
  } else if (_sensorOk) {
    configure(_atime, _gain);
  }
  return _sensorOk;
}

void ColorClassifier::setLed(bool on) { digitalWrite(_ledPin, on ? HIGH : LOW); }

void ColorClassifier::configure(uint8_t atime, uint8_t gain) {
  if (gain > 3) gain = 3;
  _atime = atime;
  _gain = gain;
  if (!_sensorOk) return;
  _tcs.setIntegrationTime(atime);
  _tcs.setGain((tcs34725Gain_t)gain);
}

RawMeas ColorClassifier::measure(uint8_t navg, uint8_t nflush) {
  RawMeas m = {0, 0, 0, 0, false, false};
  if (!_sensorOk || navg == 0) return m;

  const uint32_t sat = saturation();
  uint16_t r, g, b, c;

  setLed(true);
  delay(COLOR_LED_SETTLE_MS);
  // getRawData() doc thanh ghi TRUOC roi moi cho 1 chu ky tich phan
  // (xem Adafruit_TCS34725.cpp). Vi vay cac lan doc dau co the chua ket qua
  // cua chu ky tich phan bat dau truoc khi bat LED / dat nap -> bo di.
  for (uint8_t i = 0; i < nflush; i++) _tcs.getRawData(&r, &g, &b, &c);

  uint32_t sr = 0, sg = 0, sb = 0, sc = 0;
  for (uint8_t i = 0; i < navg; i++) {
    _tcs.getRawData(&r, &g, &b, &c);
    sr += r; sg += g; sb += b; sc += c;
    if ((uint32_t)c >= sat) m.saturated = true;
  }
  setLed(false);

  m.R = (float)sr / navg;
  m.G = (float)sg / navg;
  m.B = (float)sb / navg;
  m.C = (float)sc / navg;
  m.ok = true;
  return m;
}

colormath::Result ColorClassifier::classifyRaw(float R, float G, float B, float C, bool sat) const {
  return colormath::classify(_model, R, G, B, C, sat);
}

colormath::Result ColorClassifier::classify(RawMeas* rawOut) {
  colormath::Result res;
  memset(&res, 0, sizeof(res));
  res.label = colormath::UNKNOWN;
  res.nearest = colormath::UNKNOWN;
  if (!hasModel() || !_sensorOk) return res;

  // Dam bao do dung cau hinh da dung khi thu du lieu huan luyen.
  if (_atime != _model.atime || _gain != _model.gain) configure(_model.atime, _model.gain);

  RawMeas m = measure(_model.navg, _model.nflush);
  if (rawOut) *rawOut = m;
  if (!m.ok) return res;
  return colormath::classify(_model, m.R, m.G, m.B, m.C, m.saturated);
}

bool ColorClassifier::setModel(const colormath::Model& m, bool persist, const char* src) {
  if (!colormath::modelValid(m)) return false;
  _model = m;
  _modelSrc = src;
  configure(m.atime, m.gain);
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
  if (!p.begin(NVS_NS, true)) return false;   // read-only; that bai neu namespace chua ton tai
  colormath::Model m;
  size_t n = 0;
  if (p.getBytesLength(NVS_KEY) == sizeof(m)) n = p.getBytes(NVS_KEY, &m, sizeof(m));
  p.end();
  if (n != sizeof(m) || !colormath::modelValid(m)) return false;
  _model = m;
  _modelSrc = "nvs";
  return true;
}
