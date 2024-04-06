#include "./lib/kinematics/kinematics.hpp"
#include "./lib/PWMServo/pwmservo.hpp"
#include "./lib/SerialServo/servo.hpp"

// Joint 0, 1, 2
serialservo SerialServo_(Serial2);
// Joint 3
pwmservo PWMServo0(15,0);
// Joint 4
pwmservo PWMServo1(16,1);
// Griper 
pwmservo PWMServo2(17,2);
pwmservo PWMServo3(18,3);

// 5 DOF kinematics
kinematics Kinematics(20, 20, 20, 20, 20, 20);



void setup(){
    
}

void loop(){

}