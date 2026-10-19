// sorter.h - chu trinh phan loai mau (de cuong, muc "Chu trinh phan loai mau"):
//   NO_CAP  -> khong co nap, tiep tuc cho
//   UNKNOWN -> gap vao khu vuc "vut bo" + bao loi qua Serial
//   GREEN / RED / YELLOW -> gap vao hop tuong ung
// Bat/tat bang lenh Serial "AUTO 1" / "AUTO 0" (mac dinh tat de hieu chuan khong bi chen ngang).
#pragma once
#include "ColorClassifier.h"
#include "arm.h"

class Sorter {
 public:
  Sorter(ColorClassifier& color, Arm& arm) : _color(color), _arm(arm) {}
  void setAuto(bool on);
  bool isAuto() const { return _auto; }
  void update();   // goi trong loop()
  uint32_t count(Bin b) const { return _count[(int)b]; }

 private:
  ColorClassifier& _color;
  Arm& _arm;
  bool _auto = false;
  uint32_t _lastMs = 0;
  uint32_t _count[4] = {0, 0, 0, 0};
  static const uint32_t POLL_MS = 500;   // chu ky kiem tra tram khi khong co nap
};
