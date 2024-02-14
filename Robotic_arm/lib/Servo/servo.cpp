#include "servo.h"
enum command {
  INST_PING = 0x01,
  INST_READ = 0x02,
  INST_WRITE = 0x03,
  INST_REG_WRITE = 0x04,
  INST_ACTION = 0x05,
  INST_RESET = 0x06,
  INST_SYNC_WRITE = 0x83,
  P_PRESENT_POSITION_L = 56,
  P_PRESENT_POSITION_H = 57,
  P_PRESENT_SPEED_L = 58,
  P_PRESENT_SPEED_H = 59,
  P_PRESENT_LOAD_L = 60,
  P_PRESENT_LOAD_H = 61,
  P_PRESENT_VOLTAGE = 62,
  P_PRESENT_TEMPERATURE = 63,
  P_REGISTERED_INSTRUCTION = 64,
  P_MOVING = 66,
  P_ID = 5,

  P_VERSION_L = 3,
  P_VERSION_H = 4,

  P_BAUD_RATE = 6,
  P_RETURN_DELAY_TIME = 7,
  P_RETURN_LEVEL = 8,
  P_MIN_ANGLE_LIMIT_L = 9,
  P_MIN_ANGLE_LIMIT_H = 10,
  P_MAX_ANGLE_LIMIT_L = 11,
  P_MAX_ANGLE_LIMIT_H = 12,
  P_LIMIT_TEMPERATURE = 13,
  P_MAX_LIMIT_VOLTAGE = 14,
  P_MIN_LIMIT_VOLTAGE = 15,
  P_MAX_TORQUE_L = 16,
  P_MAX_TORQUE_H = 17,
  P_ALARM_LED = 19,
  P_ALARM_SHUTDOWN = 20,
  P_COMPLIANCE_P = 21,
  P_PUNCH_L = 24,
  P_PUNCH_H = 25,
  P_CW_DEAD = 26,
  P_CCW_DEAD = 27,

  P_TORQUE_ENABLE = 40,
  P_LED = 41,
  P_GOAL_POSITION_L = 42,
  P_GOAL_POSITION_H = 43,
  P_GOAL_TIME_L = 44,
  P_GOAL_TIME_H = 45,
  P_GOAL_SPEED_L = 46,
  P_GOAL_SPEED_H = 47,
  P_LOCK = 48  //THIS IS THE EEPROM LOCK :D SET TO TO 0 TO OPEN
};
/**
 * @brief ServoDriver class, drive the digital servo
 * 
 * 
 * @param s2 serial port for communication with the servo
 * 
 */
class ServoDriver {
private:
  HardwareSerial &serial2;
  unsigned char CheckSum = 0;
  unsigned char buffer[11];
  unsigned char act[6] = { 0xff, 0xff, 0xfe, 0x02, 0x05, 0xfa };

  unsigned char tOn[9] = { 0xff, 0xff, 0x01, 0x05, 0x03, 0x18, 0x01, 0x01, 0xDC };
  unsigned char data;
  unsigned char writeReg(int addr, int reg, unsigned char regVal) {
    buffer[0] = 0xff;  //header
    buffer[1] = 0xff;
    buffer[2] = addr;  //ID
    buffer[3] = 0x04;  //Len
    buffer[4] = 0x03;  //Write
    buffer[5] = reg;   //register
    buffer[6] = regVal;

    CheckSum = 0;

    for (int i = 2; i < (7); i++) {
      CheckSum += buffer[i];
    }

    //serial.println(CheckSum);
    CheckSum = (~CheckSum);


    serial2.write(buffer, 7);
    serial2.write(CheckSum);


    int i = 0;
    unsigned char output = -1;

    delay(10);

    while (serial2.available() > 0) {

      data = serial2.read();
      if (i == 4) {
        output = data;
      }
      i++;
    }

    return output;
  }
  void writeHL(int ID, int lReg, uint16_t value) {
    unsigned char lBit = value & 0x00FF;
    unsigned char hBit = (value & 0xFF00) >> 8;
    writeTwoReg(ID, lReg, hBit, lBit);
  }
  unsigned char writeTwoReg(int addr, int reg, unsigned char regVal1, unsigned char regVal2) {
    buffer[0] = 0xff;  //header
    buffer[1] = 0xff;
    buffer[2] = addr;  //ID
    buffer[3] = 0x05;  //Len
    buffer[4] = 0x03;  //Write
    buffer[5] = reg;   //register
    buffer[6] = regVal1;
    buffer[7] = regVal2;

    CheckSum = 0;

    for (int i = 2; i < (8); i++) {
      CheckSum += buffer[i];
    }
    CheckSum = (~CheckSum);


    serial2.write(buffer, 8);
    serial2.write(CheckSum);

    CheckSum = ~CheckSum;


    int i = 0;
    unsigned char output = -1;

    delay(10);

    while (serial2.available() > 0) {

      data = serial2.read();
      if (i == 4) {
        output = data;
      }
      i++;
    }

    return output;
  }
  unsigned char readReg(int addr, int reg) {
    buffer[0] = 0xff;
    buffer[1] = 0xff;
    buffer[2] = addr;
    buffer[3] = 0x04;
    buffer[4] = 0x02;
    buffer[5] = reg;  //register
    buffer[6] = 0x01;

    CheckSum = 0;

    for (int i = 2; i < (6); i++) {
      CheckSum += buffer[i];
    }
    CheckSum = (~CheckSum) - 1;


    serial2.write(buffer, 7);
    serial2.write(CheckSum);


    int i = 0;
    unsigned char output = -1;

    //delay(100);

    while (serial2.available() > 0) {

      data = serial2.read();

      if (i == 5) {
        output = data;
      }
      i++;
    }

    return output;
  }
public:
  ServoDriver(HardwareSerial &s2) :serial2(s2){

    serial2.begin(1000000);
  }
  //only use when it is necessary to set the ID for servo
  void setID(int originID,int targetID){
    writeReg(originID, P_LOCK, 0);

    writeReg(originID, P_ID, targetID);

    delay(100);

    writeReg(targetID, P_LOCK, 1);
  }
  bool moveTo(int ID, double position) {
    writeReg(ID, P_TORQUE_ENABLE, 1);
    position = position / (2 * PI);  //Normalize to fractions of a circle
    position = position * 1800;
    if (position < 0 || position > 1000) {
      return 0;
      //Serial.println("Failed");
    }
    int goalPos = position;
    //Serial.println(goalPos);
    writeHL(ID, P_GOAL_POSITION_L, goalPos);
    return 1;
  }
};
ServoDriver driver1(Serial2);
/**
 * @brief program test
 * 
 */


void setup() {
  

  // put your setup code here, to run once:

}
double angleR;
int angle;
int direction = 1;
void loop() {
 
  angleR = angle * PI / 180;
  if (angle >= 180) {
    direction = -1;
  } else if (angle <= 0) {
    direction = 1;
  }
  angle += direction;
  driver1.moveTo(1,angleR);
  // put your main code here, to run repeatedly:

}

