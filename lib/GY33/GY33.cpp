#include "GY33.h"

bool GY33::begin(TwoWire& wire) {
  _wire = &wire;
  return present();
}

bool GY33::present() {
  if (!_wire) return false;
  _wire->beginTransmission(_addr);
  return _wire->endTransmission() == 0;
}

// Ghi dia chi thanh ghi, repeated start, doc len byte (nhu iic_read() cua ma mau hang).
bool GY33::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  if (!_wire) return false;
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  // Overload (uint8_t, size_t, bool) co ca o Arduino-ESP32 core 2.x va 3.x.
  if (_wire->requestFrom(_addr, (size_t)len, true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
  return true;
}

bool GY33::readRaw(uint16_t rgbc[4]) {
  uint8_t b[8];
  if (!readRegs(0x00, b, 8)) return false;
  for (int i = 0; i < 4; i++) rgbc[i] = (uint16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  return true;
}

int GY33::readLedLevel() {
  uint8_t v;
  if (!readRegs(0x10, &v, 1)) return -1;
  return v >> 4;
}

bool GY33::writeLedLevel(uint8_t level) {
  if (!_wire || level > LED_LEVEL_MAX) return false;
  _wire->beginTransmission(_addr);
  _wire->write((uint8_t)0x10);
  _wire->write((uint8_t)(level << 4));
  return _wire->endTransmission() == 0;
}
