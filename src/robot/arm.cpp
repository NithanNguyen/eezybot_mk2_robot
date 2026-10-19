#include "arm.h"
#include "board_pins.h"
#include "ServoPwm.h"

const char* binName(Bin b) {
  switch (b) {
    case Bin::GREEN:  return "GREEN";
    case Bin::RED:    return "RED";
    case Bin::YELLOW: return "YELLOW";
    case Bin::REJECT: return "REJECT";
  }
  return "?";
}

#if ARM_OUTPUT_ENABLED
static ServoPwm servos[4];
static const uint8_t SERVO_PINS[4] = {pins::SERVO_J1_BASE, pins::SERVO_J2_SHOULDER,
                                      pins::SERVO_J3_ELBOW, pins::SERVO_GRIPPER};
#endif

void Arm::begin() {
#if ARM_OUTPUT_ENABLED
  for (uint8_t i = 0; i < 4; i++) servos[i].attach(SERVO_PINS[i], i);
  goHome();
#else
  Serial.println("# arm: ARM_OUTPUT_ENABLED=0 -> chua phat xung servo (khung)");
#endif
}

void Arm::goHome() {
  Serial.println("# arm TODO: goHome()");
}

void Arm::sortTo(Bin bin) {
  Serial.printf("# arm TODO: gap nap tai tram -> tha vao hop %s -> ve Home\n", binName(bin));
}
