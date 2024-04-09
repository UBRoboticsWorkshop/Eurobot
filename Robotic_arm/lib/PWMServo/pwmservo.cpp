#include <Arduino.h>
#include "pwmservo.hpp"



void pwmservo::stop() {
  ledcWrite(_ledCH, 0);
}

void pwmservo::moveTo(float angle){
  angle += _offset;
  while (angle < 0.0f) angle += 360.0f;
  while (angle > 360.0f) angle -= 360.0f;

  if (angle < _lowerAngle || angle > _upperAngle){
    log_w("Goto: %d deg, OUT OF RANGE!", uint16_t(angle));
  } else {
    uint16_t pwm = _x * angle + _lowerPWM;
    log_i("Goto: %f deg, %d pwm", angle, pwm);
    ledcWrite(_ledCH, pwm);
  }
}