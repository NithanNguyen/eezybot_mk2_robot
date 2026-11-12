// arm.h - dieu khien tay may: task FreeRTOS chu ky 20 ms chay lib/Motion/ArmMotion.h (DoubleS1D + Mk2Kinematics).
// Thong so dong hoc / quy dao: include/arm_config.h (sinh tu eezybot_mk2_sim/params.py, khong sua tay).
//
// CHAY KHO (ARM_OUTPUT_ENABLED = 0, mac dinh): tinh day du goc lenh moi nhip nhung KHONG phat xung servo.
// Bat ARM_OUTPUT_ENABLED = 1 se bao loi bien dich cho toi khi co hieu chuan goc servo -> do rong xung (buoc 6).
//
// Lenh Serial (main.cpp chuyen cac dong bat dau bang "ARM"):
//   ARM SORT <GREEN|RED|WHITE|REJECT>  chay mot chu trinh gap -> tha -> ve Home (khong can cam bien)
//   ARM HOME                           ve Home roi mo kep
//   ARM TRACE 1 | ARM TRACE 0          in / tat moi nhip: "CMD,<nhip>,<pha>,<servo0>,<servo1>,<servo2>,<kep>" (do)
//   ARM TIMING [RESET]                 thong ke thoi gian tinh moi nhip va do lech nhip 20 ms
//   ARM STATUS                         trang thai, goc lenh hien tai
// Ket thuc moi chu trinh in "ARM DONE <ten> status=<OK|loi IK/gioi han> ticks=<n> cmax_us=<us> dropped=<n>".
// tools/motion/test_motion_parity.py --port <cong> dung cac lenh nay de so trace ESP32 voi mo phong.
#pragma once
#include <Arduino.h>

#ifndef ARM_OUTPUT_ENABLED
#define ARM_OUTPUT_ENABLED 0
#endif

enum class Bin : uint8_t { GREEN = 0, RED = 1, WHITE = 2, REJECT = 3 };
const char* binName(Bin b);

class Arm {
 public:
  void begin();             // tao task 20 ms; coi tay dang o Home (xem README: dat tay ve Home truoc khi cap nguon)
  bool busy() const;        // co yeu cau dang cho hoac dang chay chu trinh
  void goHome();
  // De cuong: den tren diem gap -> ha thang -> dong kep -> nhac len -> den diem tha (day nap cach mat hop 50 mm)
  // -> mo kep -> ve Home. Vung vut bo: den phia tren roi ha thang.
  void sortTo(Bin bin);
  void update();            // goi trong loop(): in vet nhip va ket qua tu task
  bool handleCommand(char* tok[], int n);  // true neu la lenh "ARM ..."
};
