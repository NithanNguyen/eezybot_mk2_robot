// arm.h - giao dien dieu khien tay may. HIEN LA KHUNG:
// chua phat xung servo (ARM_OUTPUT_ENABLED = 0) cho toi khi co Home, FK/IK va quy dao.
// Thong so dong hoc / quy dao theo de cuong: include/arm_config.h.
#pragma once
#include <Arduino.h>

#ifndef ARM_OUTPUT_ENABLED
#define ARM_OUTPUT_ENABLED 0
#endif

enum class Bin : uint8_t { GREEN = 0, RED = 1, WHITE = 2, REJECT = 3 };
const char* binName(Bin b);

class Arm {
 public:
  void begin();
  bool busy() const { return false; }   // TODO: true khi dang chay quy dao
  void goHome();                         // TODO
  // De cuong: den tren diem gap -> ha thang -> dong kep -> nhac len ->
  // den tren hop dich 50 mm -> mo kep -> ve Home.
  void sortTo(Bin bin);                  // TODO
};
