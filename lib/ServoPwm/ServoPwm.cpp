#include "ServoPwm.h"

void ServoPwm::attach(uint8_t pin, uint8_t channel, uint16_t minUs, uint16_t maxUs) {
  _pin = pin;
  _channel = channel;
  _minUs = minUs;
  _maxUs = maxUs;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(_pin, PWM_HZ, PWM_BITS);
#else
  ledcSetup(_channel, PWM_HZ, PWM_BITS);
  ledcAttachPin(_pin, _channel);
#endif
}

void ServoPwm::writeUs(uint32_t us) {
  if (_pin == 255) return;
  if (us < _minUs) us = _minUs;
  if (us > _maxUs) us = _maxUs;
  _lastUs = us;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(_pin, usToDuty(us));
#else
  ledcWrite(_channel, usToDuty(us));
#endif
}
