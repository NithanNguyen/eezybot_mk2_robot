// servo_center: phat xung 50 Hz, do rong 1500 us lien tuc cho 4 servo.
// Dung de dua truc servo ve vi tri giua TRUOC KHI cam horn.
//   pio run -e servo_center -t upload
#include <Arduino.h>
#include "board_pins.h"
#include "ServoPwm.h"

static const uint8_t PINS[4] = {pins::SERVO_J1_BASE, pins::SERVO_J2_SHOULDER,
                                pins::SERVO_J3_ELBOW, pins::SERVO_GRIPPER};
static const char* NAMES[4] = {"J1 de", "J2 vai", "J3 khuyu", "SG90 kep"};
static const uint32_t CENTER_US = 1500;

static ServoPwm servos[4];

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; i++) {
    servos[i].attach(PINS[i], i);
    servos[i].writeUs(CENTER_US);
  }
  Serial.printf("# servo_center: %u us, %u Hz, duty=%u/%u\n", (unsigned)CENTER_US,
                (unsigned)ServoPwm::PWM_HZ, (unsigned)ServoPwm::usToDuty(CENTER_US),
                (unsigned)((1UL << ServoPwm::PWM_BITS) - 1));
  for (uint8_t i = 0; i < 4; i++) Serial.printf("#   GPIO%u -> %s\n", PINS[i], NAMES[i]);
}

void loop() {
  // LEDC tu phat xung lien tuc; chi in nhip song de biet ESP32 khong bi reset.
  static uint32_t t = 0;
  if (millis() - t >= 2000) {
    t = millis();
    Serial.printf("# alive %lu s\n", (unsigned long)(millis() / 1000));
  }
}
