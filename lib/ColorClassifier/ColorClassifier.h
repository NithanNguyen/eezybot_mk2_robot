// ColorClassifier.h - do mau bang GY-33 + phan loai theo model luu trong NVS.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "GY33.h"
#include "ColorMath.h"

struct RawMeas {
  float R, G, B, C;   // trung binh navg lan doc
  bool saturated;     // co kenh nao cham 65535
  bool ok;            // false neu loi I2C
};

class ColorClassifier {
 public:
  // Khoang cach giua hai lan doc: dai hon chu ky cap nhat ~100 ms cua GY-33.
  static const uint32_t READ_INTERVAL_MS = 120;

  ColorClassifier(uint8_t sdaPin, uint8_t sclPin, uint32_t i2cHz);

  bool begin();                       // khoi dong I2C, GY-33, nap model tu NVS
  bool sensorOk() const { return _sensorOk; }
  int ledLevel() { return _gy.readLedLevel(); }
  bool setLedLevel(uint8_t level) { return _gy.writeLedLevel(level); }

  RawMeas measure(uint8_t navg, uint16_t settleMs);

  // Do theo cau hinh cua model roi phan loai.
  colormath::Result classify(RawMeas* rawOut = nullptr);
  colormath::Result classifyRaw(float R, float G, float B, float C, bool sat) const {
    return colormath::classify(_model, R, G, B, C, sat);
  }

  bool hasModel() const { return colormath::modelValid(_model); }
  const colormath::Model& model() const { return _model; }
  const char* modelSource() const { return _modelSrc; }
  bool setModel(const colormath::Model& m, bool persist, const char* src);
  void clearModel();

 private:
  bool loadModelFromNvs();
  void ensureModelLed();

  GY33 _gy;
  uint8_t _sda, _scl;
  uint32_t _hz;
  bool _sensorOk = false;
  colormath::Model _model;
  const char* _modelSrc = "none";
};
