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
} Position;

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
  shell_println("================================help===============================");
  shell_println("== 0. Set Joints' Angle: goto <J0> <J1> <J2> <J3> <J4> <J5>      ==");
  shell_println("== 1. Turn Certain Angle: add <J0> <J1> <J2> <J3> <J4> <J5>      ==");
  shell_println("== 2. Inverse Kinematics (base): inv_kin <x> <y> <z> -r(rightly) ==");
  shell_println("== 3. Inverse Kinematics (top): inv_kin_top <x> <y> <z> -r       ==");
  shell_println("== 4. Forward Kinematics: for_kin                                ==");
  shell_println("== 5. Current Joint Angle: angle                                 ==");
  shell_println("== 6. EStop: estop                                               ==");
  shell_println("== 7. Reboot: reboot                                             ==");
  shell_println("===================================================================");

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
  shell_printf("Current joint angle: %f, %f, %f, %f, %f, %f, %f \n", Position.joint0, Position.joint1, Position.joint2, Position.joint3, Position.joint4, Position.gripper0, Position.gripper1);

  return SHELL_RET_SUCCESS;
}

int For_kin(int argc, char** argv){
  Kinematics.ForwardKinematics(Position.joint0, Position.joint1, Position.joint2, Position.joint3, Position.joint4);
  shell_printf("Current endpoint position: X %f, Y %f, Z %f \n", Kinematics.ForwardKinematics_.X, Kinematics.ForwardKinematics_.Y, Kinematics.ForwardKinematics_.Z);

  return SHELL_RET_SUCCESS;
}

int Inv_kin(int argc, char** argv){
  bool r = false;

  if(argc < 5){
    shell_print_error(E_SHELL_ERR_ARGCOUNT,0);
    shell_println("");
    return SHELL_RET_FAILURE;
  } else if ((argc == 6) && (!strcmp(argv[5], (const char *) "-r"))){
    r = true;
  }

  float X = strtof(argv[1], 0);
  float Y = strtof(argv[2], 0);
  float Z = strtof(argv[3], 0);
  float endeffectorAngle = strtof(argv[4], 0);
  ESP_LOGD("Shell", "Position: X %f, Y %f, Z %f, Ang %f, rightly %d", X, Y, Z, endeffectorAngle, r);

  // calculate
  Kinematics.InverseKinematics(X, Y, Z, endeffectorAngle, r);
  ESP_LOGI("Shell", "inv_kin: J0 %f, J1 %f, J2 %f, J3 %f, J4 %f", Kinematics.Position_.Joint0, Kinematics.Position_.Joint1, Kinematics.Position_.Joint2, Kinematics.Position_.Joint3, Kinematics.Position_.Joint0);

  SerialServo.moveTo(1, Kinematics.Position_.Joint0);
  SerialServo.moveTo(2, Kinematics.Position_.Joint1);
  SerialServo.moveTo(3, Kinematics.Position_.Joint2);

  PWMServo0.moveTo(Kinematics.Position_.Joint3);
  PWMServo1.moveTo(Kinematics.Position_.Joint0);

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

void setup() {
  SerialServo.enabletorque(254, true); // for all
  SerialServo.moveTo(1, 0);
  SerialServo.moveTo(2, 0);
  SerialServo.moveTo(3, 0);

  PWMServo0.moveTo(0);
  PWMServo1.moveTo(0);

  PWMServo2.moveTo(0);
  PWMServo3.moveTo(0);

  delay(100);

  // shell init
  Serial.begin(115200);
  shell_init(shell_reader, shell_writer, "5-DOF ARM, type 'help/r/n' for more info.");
  shell_register(ESTOP, "estop");
  shell_register(help, "help");
  shell_register(Reboot, "reboot");
  shell_register(Angle, "angle");
  shell_register(For_kin, "for_kin");
  shell_register(Inv_kin, "inv_kin");
  shell_register(Add, "add");
  shell_register(Goto, "goto"); 
  shell_register(Inv_kin_top, "inv_kin_top");
}

void loop(){
  // shell task
  shell_task();
}
