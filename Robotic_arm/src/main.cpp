#include "kinematics.hpp"
#include "pwmservo.hpp"
#include "servo.hpp"
#include <Arduino.h>

// Joint 0, 1, 2
serialservo SerialServo(Serial2);
// Joint 3
pwmservo PWMServo0(15,0);
// Joint 4
pwmservo PWMServo1(16,1);
// Gripper 
pwmservo PWMServo2(17,2);
pwmservo PWMServo3(18,3);

// 5 DOF kinematics
kinematics Kinematics(20, 20, 20, 20, 20, 20);



void setup(){
  SerialServo.enabletorque(1);

  SerialServo.moveTo(1, 90);
  PWMServo1.servoSetAngle(90);
}

void loop(){

}