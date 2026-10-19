// arm.h - giao dien dieu khien tay may. HIEN LA KHUNG (stub):
// cac ham chi in ra Serial, CHUA phat xung servo (ARM_OUTPUT_ENABLED = 0),
// de tranh tay may chuyen dong khi chua co vi tri Home, dong hoc FK/IK va quy dao.
//
// Viec can lam sau (theo de cuong): Home, FK/IK, quy dao double-S,
// hieu chuan us <-> do cho tung servo, gioi han khop.
#pragma once
#include <Arduino.h>

#ifndef ARM_OUTPUT_ENABLED
#define ARM_OUTPUT_ENABLED 0
#endif

enum class Bin : uint8_t { GREEN = 0, RED = 1, YELLOW = 2, REJECT = 3 };
const char* binName(Bin b);

class Arm {
 public:
  void begin();
  bool busy() const { return false; }   // TODO: true khi dang chay quy dao
  void goHome();                         // TODO
  // Chu trinh gap nap tai tram cam bien roi tha vao hop 'bin', ve Home.
  void sortTo(Bin bin);                  // TODO
};
