// ColorClassifier.h - doc TCS34725 + phan loai mau nap, dung chung cho
// firmware tram hieu chuan (src/main.cpp) va firmware tay may sau nay.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "ColorMath.h"

struct RawMeas {
  float R, G, B, C;   // trung binh navg lan doc
  bool saturated;     // co lan doc nao cham muc bao hoa
  bool ok;            // false neu khong giao tiep duoc cam bien
};

class ColorClassifier {
 public:
  // ledPin: chan dieu khien LED cua module (HIGH = bat)
  ColorClassifier(uint8_t ledPin, uint8_t sdaPin = 21, uint8_t sclPin = 22);

  // Khoi dong I2C + cam bien, nap model tu NVS (neu co). Tra ve true neu thay cam bien.
  bool begin();
  bool sensorOk() const { return _sensorOk; }

  void setLed(bool on);
  void configure(uint8_t atime, uint8_t gain);   // gain: 0..3
  uint8_t atime() const { return _atime; }
  uint8_t gain() const { return _gain; }
  uint32_t saturation() const { return colormath::saturationLevel(_atime); }

  // Bat LED, bo nflush lan doc dau, lay trung binh navg lan, tat LED.
  RawMeas measure(uint8_t navg, uint8_t nflush);

  // Do theo cau hinh cua model roi phan loai (dung trong chu trinh gap - tha).
  colormath::Result classify(RawMeas* rawOut = nullptr);
  colormath::Result classifyRaw(float R, float G, float B, float C, bool sat) const;

  bool hasModel() const { return colormath::modelValid(_model); }
  const colormath::Model& model() const { return _model; }
  const char* modelSource() const { return _modelSrc; }

  // persist = true: luu vao NVS (con sau khi mat dien / nap lai firmware khac phan vung app)
  bool setModel(const colormath::Model& m, bool persist, const char* src = "nvs");
  void clearModel();   // xoa model trong NVS va RAM

 private:
  bool loadModelFromNvs();

  Adafruit_TCS34725 _tcs;
  uint8_t _ledPin, _sda, _scl;
  uint8_t _atime, _gain;
  bool _sensorOk;
  colormath::Model _model;
  const char* _modelSrc;
};
