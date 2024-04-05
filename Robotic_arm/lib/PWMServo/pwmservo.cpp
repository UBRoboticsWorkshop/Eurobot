#include <Arduino.h>
#include "pwmservo.hpp"



void pwmservo::servoStop() {
  ledcWrite(_ledCH, 0);
}

void pwmservo::servoGo(float angele){



  ledcWrite(_ledCH, pwm);
} 
