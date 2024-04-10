#include "kinematics.hpp"
#include "pwmservo.hpp"
#include "servo.hpp"
#include "Shell.h"
#include <Arduino.h>



// Joint 0, 1, 2
serialservo SerialServo(Serial2);
// Joint 3
pwmservo PWMServo0(25, 0, 135, 900, 8500, 0, 210);
// Joint 4
pwmservo PWMServo1(26, 1);
// Gripper 
pwmservo PWMServo2(27, 2, 0);
pwmservo PWMServo3(13, 3, 0);
// 5 DOF kinematics
kinematics Kinematics(100, 100, 45); //mm

struct position {
  float gripper0, gripper1;
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

void moveJoint(){
  SerialServo.moveTo(1, Kinematics.Position_.Joint0);
  SerialServo.moveTo(2, Kinematics.Position_.Joint1, 174);
  SerialServo.moveTo(3, Kinematics.Position_.Joint2, 184);
  PWMServo0.moveTo(Kinematics.Position_.Joint3);
  PWMServo1.moveTo(Kinematics.Position_.Joint4);
}

int help(int argc, char** argv){
  shell_println("====================================help===================================");
  shell_println("== 0. Set Joints' Angle: goto <J0> <J1> <J2> <J3> <J4> <J5> -add         ==");
  shell_println("== 2. Inverse Kinematics (base): inv_kin <x> <y> <z> -r(rightly) -t(top) ==");
  shell_println("== 4. Forward Kinematics: for_kin                                        ==");
  shell_println("== 5. Current Joint Angle: angle                                         ==");
  shell_println("== 6. EStop: estop                                                       ==");
  shell_println("== 7. Reboot: reboot                                                     ==");
  shell_println("===========================================================================");

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
  Serial.printf("Current joint angle: %f, %f, %f, %f, %f, %f, %f \n", Kinematics.Position_.Joint0, Kinematics.Position_.Joint1, Kinematics.Position_.Joint2, Kinematics.Position_.Joint3, Kinematics.Position_.Joint4, Position.gripper0, Position.gripper1);

  return SHELL_RET_SUCCESS;
}

int For_kin(int argc, char** argv){
  Kinematics.ForwardKinematics();
  Serial.printf("Current endpoint position: X %f, Y %f, Z %f \n", Kinematics.ForwardKinematics_.X, Kinematics.ForwardKinematics_.Y, Kinematics.ForwardKinematics_.Z);

  return SHELL_RET_SUCCESS;
}

int Inv_kin(int argc, char** argv){
  bool r = false;

  if(argc < 5){
    shell_print_error(E_SHELL_ERR_ARGCOUNT, 0);
    shell_println("");
    return SHELL_RET_FAILURE;
  } else if ((argc >= 6) && (!strcmp(argv[5], (const char *) "-r"))){
    r = true;
  }
  // Kinematics.ForwardKinematics();
  // Kinematics.CoordinateTrans(-Kinematics.ForwardKinematics_.X, -Kinematics.ForwardKinematics_.Y, -Kinematics.ForwardKinematics_.Z, Target.X, Target.Y, Target.Z);

  float X = strtof(argv[1], 0);
  float Y = strtof(argv[2], 0);
  float Z = strtof(argv[3], 0);
  float endeffectorAngle = strtof(argv[4], 0);
  ESP_LOGV("Shell", "Position: X %f, Y %f, Z %f, Ang %f, rightly %d", X, Y, Z, endeffectorAngle, r);

  // calculate
  if (Kinematics.InverseKinematics(X, Y, Z, endeffectorAngle, r)){
    ESP_LOGI("Shell", "inv_kin: J0 %f, J1 %f, J2 %f, J3 %f, J4 %f", Kinematics.Position_.Joint0, Kinematics.Position_.Joint1, Kinematics.Position_.Joint2, Kinematics.Position_.Joint3, Kinematics.Position_.Joint0);
    moveJoint();
    return SHELL_RET_SUCCESS;
  } else {
    shell_print_error(E_SHELL_ERR_ACTION, 0);
    shell_println("");
    return SHELL_RET_FAILURE;
  }
}

int Goto(int argc, char** argv){
  if(argc==6){
    Kinematics.Position_.Joint0 = strtof(argv[1], 0);
    Kinematics.Position_.Joint1 = strtof(argv[2], 0);
    Kinematics.Position_.Joint2 = strtof(argv[3], 0);
    Kinematics.Position_.Joint3 = strtof(argv[4], 0);
    Kinematics.Position_.Joint4 = strtof(argv[5], 0);
    moveJoint();
    //Position.gripper0 = 0;
  } else if ((argc == 7) && (!strcmp(argv[6], (const char *) "-add"))){
    Kinematics.Position_.Joint0 += strtof(argv[1], 0);
    Kinematics.Position_.Joint1 += strtof(argv[2], 0);
    Kinematics.Position_.Joint2 += strtof(argv[3], 0);
    Kinematics.Position_.Joint3 += strtof(argv[4], 0);
    Kinematics.Position_.Joint4 += strtof(argv[5], 0);
    moveJoint();
  } else {
    shell_print_error(E_SHELL_ERR_ARGCOUNT,0);
    shell_println("");
    return SHELL_RET_FAILURE;
  }
  return SHELL_RET_SUCCESS;
}

void setup() {
  SerialServo.enabletorque(254, true); // for all
  Kinematics.Position_.Joint0 = Kinematics.Position_.Joint1 = Kinematics.Position_.Joint2 = Kinematics.Position_.Joint3 = Kinematics.Position_.Joint4 = 0.0f;
  moveJoint();

  PWMServo2.moveTo(0);
  PWMServo3.moveTo(0);

  // shell init
  Serial.begin(115200);
  shell_init(shell_reader, shell_writer, "5-DOF ARM, type 'help/r/n' for more info.");
  shell_register(ESTOP, "estop");
  shell_register(help, "help");
  shell_register(Reboot, "reboot");
  shell_register(Angle, "angle");
  shell_register(For_kin, "for_kin");
  shell_register(Inv_kin, "inv_kin");
  shell_register(Goto, "goto"); 
}

void loop(){
  // shell task
  shell_task();
}
