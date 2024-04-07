#include <Arduino.h>
#include "pwmservo.hpp"



void pwmservo::servoStop() {
  ledcWrite(_ledCH, 0);
}

void pwmservo::servoGoto(uint16_t angle){ // processed angle lower - upper
  uint16_t pwm = map(angle, _lowerAngle, _upperAngle, _lowerPWM, _upperPWM);
  ledcWrite(_ledCH, pwm);
}

void pwmservo::moveTo(float angle){
  angle += _offset;
  if (angle < 0.0f) {
    angle += 360.0f;
  } else if (angle > 360.0f) {
    angle = int(angle) % 360;
  }

  if (angle < _lowerAngle || angle > _upperAngle){
    ESP_LOGW("pwmservo", "OUT OF RANGE!");
  } else {
    servoGoto(angle);
  }
}