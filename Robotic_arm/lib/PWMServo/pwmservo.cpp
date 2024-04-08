#include <Arduino.h>
#include "pwmservo.hpp"



void pwmservo::stop() {
  ledcWrite(_ledCH, 0);
}

void pwmservo::servoGoto(uint16_t angle){ // processed angle lower - upper
  uint16_t pwm = map(angle, _lowerAngle, _upperAngle, _lowerPWM, _upperPWM);
  ESP_LOGI("pwmservo", "Goto: %d deg, %d pwm", angle, pwm);
  ledcWrite(_ledCH, pwm);
}

void pwmservo::moveTo(float angle){
  angle += _offset;
  while (angle < 0.0f) angle += 360;
  while (angle > 360.0f) angle -= 360;

  if (angle < _lowerAngle || angle > _upperAngle){
    ESP_LOGW("pwmservo", "Goto: %d deg, OUT OF RANGE!", uint16_t(angle));
  } else {
    servoGoto(angle);
  }
}