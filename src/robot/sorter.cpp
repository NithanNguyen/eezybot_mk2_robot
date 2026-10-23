#include "sorter.h"
#include "color_console.h"

void Sorter::setAuto(bool on) {
  if (on && (!_color.sensorOk() || !_color.hasModel())) {
    // Khong co cam bien/model thi khong biet co nap hay khong -> khong cho tay may chay.
    Serial.printf("ERR AUTO_REFUSED sensor=%d model=%d\n", _color.sensorOk() ? 1 : 0,
                  _color.hasModel() ? 1 : 0);
    _auto = false;
    return;
  }
  _auto = on;
  Serial.printf("OK AUTO %d\n", on ? 1 : 0);
  if (on) _arm.goHome();
}

void Sorter::update() {
  if (!_auto || _arm.busy() || millis() - _lastMs < POLL_MS) return;
  _lastMs = millis();

  // In dong CLS; neu UNKNOWN thi in them "# ERR UNKNOWN_COLOR ..." (bao loi qua Serial).
  colormath::Result r = color_console::classifyAndPrint();
  Bin bin;
  switch (r.label) {
    case colormath::NO_CAP:  return;                     // khong co nap: tiep tuc cho
    case colormath::GREEN:   bin = Bin::GREEN;  break;
    case colormath::RED:     bin = Bin::RED;    break;
    case colormath::WHITE:   bin = Bin::WHITE;  break;
    default:                 bin = Bin::REJECT; break;   // UNKNOWN -> khu vuc vut bo
  }
  _count[(int)bin]++;
  Serial.printf("# SORT %s (lan %lu)\n", binName(bin), (unsigned long)_count[(int)bin]);
  _arm.sortTo(bin);
}
