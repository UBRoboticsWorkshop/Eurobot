#include "kinematics.hpp"
#include "pwmservo.hpp"
#include "servo.hpp"
#include <Arduino.h>
// #include <SCServo.h>
// SMS_STS st;
// Joint 0, 1, 2
serialservo SerialServo(Serial2);
// Joint 3
pwmservo PWMServo0(25, 0);
// Joint 4
pwmservo PWMServo1(26, 1);
// Gripper 
pwmservo PWMServo2(27, 2, 0);
pwmservo PWMServo3(13, 3, 0);

// 5 DOF kinematics
kinematics Kinematics(20, 20, 20, 20, 20, 20);



void setup() {
  SerialServo.enabletorque(254); // for all
  SerialServo.moveTo(1, 0);
  SerialServo.moveTo(2, 0);
  SerialServo.moveTo(3, 0);

  PWMServo0.moveTo(-90);
  PWMServo1.moveTo(0);

  PWMServo2.moveTo(0);
  PWMServo3.moveTo(0);
}

int angle= -91;
void loop(){
  if (angle > 360) angle = -91;
  PWMServo0.moveTo(angle++);
  delay(100);
}