#include "kinematics.hpp"
#include "pwmservo.hpp"
#include "servo.hpp"
#include "Shell.h"
#include <Arduino.h>



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

struct position {
  float joint0, joint1, joint2, joint3, joint4, gripper0, gripper1;
  float X, Y, Z, A, B, C;
} Position;



void setup() {
  Serial.begin(115200);

  SerialServo.enabletorque(254, true); // for all
  SerialServo.moveTo(1, 0);
  SerialServo.moveTo(2, 0);
  SerialServo.moveTo(3, 0);

  PWMServo0.moveTo(0);
  PWMServo1.moveTo(0);

  PWMServo2.moveTo(0);
  PWMServo3.moveTo(0);

  // shell init
  shell_init(shell_reader, shell_writer, "5-DOF ARM, type 'help' for more info.");
  shell_register(ESTOP, "ESTOP");
  shell_register(help, "help");
  shell_register(reboot, "Reboot");
  shell_register(angle, "Angle");
}

void loop(){
  // shell task
  shell_task();

  switch (INSTRUCTION_BUFFER[0]) {

    case 0x01:{// Idle
      if (Serial.available()) {Serial.readBytes(INSTRUCTION_BUFFER, INSTRUCTION_SIZE);}
      break;
    }

    case 0x02:{// Set Angle for each joint
      delay(100); // wait
      if (Serial.available() < (FLOAT_SIZE * 5)) {
        Serial.println("A float contains 4 bytes * 4 Joints!");
        break;
      }

      // Joint0
      Serial.readBytes(&Target.joint0, FLOAT_SIZE);
      //memcpy (&Target.joint0, FLOAT_BUFFER, 4);

      // Joint1
      Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
      memcpy (&Target.joint1, FLOAT_BUFFER, 4);

      // Joint2
      Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
      memcpy (&Target.joint2, FLOAT_BUFFER, 4);

      // Joint3
      Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
      memcpy (&Target.joint3, FLOAT_BUFFER, 4);

      Difference.joint0 = Target.joint0 - Position.joint0;
      Difference.joint1 = Target.joint1 - Position.joint1;
      Difference.joint2 = Target.joint2 - Position.joint2;
      Difference.joint3 = Target.joint3 - Position.joint3;

      INSTRUCTION_BUFFER[0] = 0x03;
      break;
    }

    case 0x04:{// Inverse kinematics
      if (Serial.available() < 12) { // 1-4: X, 5-8: Y, 9-12: Z, 13-16: DirectionX, 17-20: DirectionY, 21-24: DirectionZ
        delay(100); // wait
        Serial.println("A float contains 4 bytes * 3 Axis!");
        break;
      }

      // X
      Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
      memcpy (&Target.X, FLOAT_BUFFER, 4);

      // Y
      Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
      memcpy (&Target.Y, FLOAT_BUFFER, 4);

      // Z
      Serial.readBytes(FLOAT_BUFFER, FLOAT_SIZE);
      memcpy (&Target.Z, FLOAT_BUFFER, 4);

      /* todo??? rotation vector
      // DirectionX, A
      // DirectionY, B
      // DirectionZ, C09
      */

      // calculate
      Kinematics.InverseKinematics(Target.X, Target.Y, Target.Z);
      
      if (Kinematics.Position_.Lefty) { // todo: consider about the angle at the end point
        Target.joint0 = Kinematics.Position_.LeftyJoint0;
        Target.joint1 = Kinematics.Position_.LeftyJoint1;
        Target.joint2 = Kinematics.Position_.LeftyJoint2;
        Target.joint3 = Kinematics.Position_.LeftyJoint3;
      } else if (Kinematics.Position_.Rightly) {
        Target.joint0 = Kinematics.Position_.RightlyJoint0;
        Target.joint1 = Kinematics.Position_.RightlyJoint1;
        Target.joint2 = Kinematics.Position_.RightlyJoint2;
        Target.joint3 = Kinematics.Position_.RightlyJoint3;
      } else {
        Serial.println("Unreachable");
        INSTRUCTION_BUFFER[0] = 0x01;
        break;
      }

      Difference.joint0 = Target.joint0 - Position.joint0;
      Difference.joint1 = Target.joint1 - Position.joint1;
      Difference.joint2 = Target.joint2 - Position.joint2;
      Difference.joint3 = Target.joint3 - Position.joint3;

      INSTRUCTION_BUFFER[0] = 0x03;
      break;
    }

    case 0x06:{ // Forward kinematics
      Kinematics.ForwardKinematics(Position.joint0, Position.joint1, Position.joint2);
      Serial.print("Current endpoint position: ");
      Serial.print("X: ");
      Serial.print(Kinematics.ForwardKinematics_.X);
      Serial.print("  Y: ");
      Serial.print(Kinematics.ForwardKinematics_.Y);
      Serial.print("  Z: ");
      Serial.print(Kinematics.ForwardKinematics_.Z);
      INSTRUCTION_BUFFER[0] = 0x01;
      break;
    }

  }
}

int shell_reader(char * data){
  // Wrapper for Serial.read() method
  if (Serial.available()) {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

void shell_writer(char data){
  // Wrapper for Serial.write() method
  Serial.write(data);
}

int help(int argc, char** argv){
  shell_println("=============================help=============================");
  shell_println("== 0. Set Joints' Angle: Goto <J0> <J1> <J2> <J3> <J4> <J5> ==");
  shell_println("== 1. Turn Certain Angle: Add <J0> <J1> <J2> <J3> <J4> <J5> ==");
  shell_println("== 2. Inverse Kinematics (base): Inv_kin <x> <y> <z>        ==");
  shell_println("== 3. Inverse Kinematics (top end): Inv_kin_top <x> <y> <z> ==");
  shell_println("== 4. Forward Kinematics: For_kin                           ==");
  shell_println("== 5. Current Joint Angle: Angle                            ==");
  shell_println("== 6. EStop: ESTOP                                          ==");
  shell_println("== 7. Reboot: Reboot                                        ==");
  shell_println("==============================================================");

  return SHELL_RET_SUCCESS;
}

int ESTOP(int argc, char** argv){
  SerialServo.enabletorque(254, false);

  PWMServo0.stop();
  PWMServo1.stop();

  PWMServo2.stop();
  PWMServo3.stop();

  shell_println("ESTOP! Reboot to deactive.");

  return SHELL_RET_SUCCESS;
}

int Reboot(int argc, char** argv){
  ESP.restart();
  return SHELL_RET_SUCCESS;
}

int Angle(int argc, char** argv){
  shell_print("Current joint angle: ");
  shell_print(Position.joint0); shell_print(", ");
  shell_print(Position.joint1); shell_print(", ");
  shell_print(Position.joint2); shell_print(", ");
  shell_print(Position.joint3); shell_print(", ");
  shell_print(Position.joint4); shell_print(", ");
  shell_print(Position.gripper0); shell_print(", ");
  shell_println(Position.gripper1);

  return SHELL_RET_SUCCESS;
}

int For_kin(int argc, char** argv){
  ESP.restart();
  return SHELL_RET_SUCCESS;
}

int Anv_kin(int argc, char** argv){
  ESP.restart();
  return SHELL_RET_SUCCESS;
}

int Add(int argc, char** argv){
  ESP.restart();
  return SHELL_RET_SUCCESS;
}

int Goto(int argc, char** argv){
  ESP.restart();
  return SHELL_RET_SUCCESS;
}

int Inv_kin_top(int argc, char** argv){
  ESP.restart();
  return SHELL_RET_SUCCESS;
}
