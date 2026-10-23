#include "arm.h"
#include "arm_config.h"
#include "board_pins.h"
#include "ServoPwm.h"

const char* binName(Bin b) {
  switch (b) {
    case Bin::GREEN:  return "GREEN";
    case Bin::RED:    return "RED";
    case Bin::WHITE:  return "WHITE";
    case Bin::REJECT: return "REJECT";
  }
  return "?";
}

#if ARM_OUTPUT_ENABLED
static ServoPwm servos[4];
#endif

void Arm::begin() {
#if ARM_OUTPUT_ENABLED
  servos[0].attach(pins::SERVO_J1_BASE, 0);
  servos[1].attach(pins::SERVO_J2_SHOULDER, 1);
  servos[2].attach(pins::SERVO_J3_ELBOW, 2);
  servos[3].attach(pins::SERVO_GRIPPER, 3, armcfg::SG90_MIN_US, armcfg::SG90_MAX_US);
  goHome();
#else
  Serial.println("# arm: ARM_OUTPUT_ENABLED=0 -> chua phat xung servo (khung)");
#endif
}

void Arm::goHome() {
  Serial.println("# arm TODO: goHome()");
}

void Arm::sortTo(Bin bin) {
  Serial.printf("# arm TODO: gap nap tai tram -> tha cach hop %s %.0f mm -> ve Home\n",
                binName(bin), armcfg::DROP_HEIGHT_ABOVE_BIN);
}
