#include "servo.hpp"

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83

//memory table definition
//-------EPROM(read only)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(read & write)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM(read & write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM(read only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70


/*
 * @brief Broadcast ID: 254 (0xFE)
*/
void serialservo::setID(uint8_t originID, uint8_t targetID){
  write8bit(originID, SMS_STS_LOCK, 0);
  write8bit(originID, SMS_STS_ID, targetID);
  write8bit(targetID, SMS_STS_LOCK, 1);
}

/*
 * @param position in deg  
 */
bool serialservo::moveTo(uint8_t ID, float position){
  if(position < 0.0f) position += 360.0f;
  if (position > 360.0f) position -= 360.0f;

  position = position / (2.0f * PI) * 1800;

  if (position < 0 || position > 1000) {
    return 0;
    ESP_LOGW("serial_servo", "OUT OF RANGE!");
  }
  ESP_LOGI("serial_servo", "Servo %d goto: %d", ID, uint16_t(position));
  write16bit(ID, SMS_STS_GOAL_POSITION_L, position);
  return 1;
}

void serialservo::enabletorque(uint8_t ID){
  write8bit(ID, SMS_STS_TORQUE_ENABLE, 1);
}

void serialservo::writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun) {
  uint8_t msgLen = 2;
  uint8_t bBuf[6];
  uint8_t CheckSum = 0;
  bBuf[0] = 0xff;
  bBuf[1] = 0xff;
  bBuf[2] = ID;
  bBuf[4] = Fun;
  if (nDat) {
    msgLen += nLen + 1;
    bBuf[3] = msgLen;
    bBuf[5] = MemAddr;
    _serial.write(bBuf, 6);

  } else {
    bBuf[3] = msgLen;
    _serial.write(bBuf, 5);
  }
  CheckSum = ID + msgLen + Fun + MemAddr;
  uint8_t i = 0;
  if (nDat) {
    for (int i = 0; i < nLen; i++) {
      CheckSum += nDat[i];
    }
    _serial.write(nDat, nLen);
  }
  _serial.write(~CheckSum);
}

void serialservo::writeTest(uint8_t ID, uint8_t reg, uint8_t *data, uint8_t nLen){
  writeBuf(ID, reg, data, nLen, INST_WRITE);
}

void serialservo::write16bit(uint8_t ID, uint8_t reg, uint16_t val){
  uint8_t data[2];
  data[1] = (uint8_t)((val&0xFF00)>>8);
  data[0] = (uint8_t)(val&0x00FF);
  ESP_LOGI("serial_servo", "Write 2-bytes: %X%X", data[0], data[1]);
  writeBuf(ID, reg, data, 2, INST_WRITE);
}

void serialservo::write8bit(uint8_t ID, uint8_t reg, uint8_t val){
  writeBuf(ID, reg, &val, 1, INST_WRITE);
}

void serialservo::readTest(uint8_t ID, uint8_t reg, uint8_t *data, uint8_t nLen) {
  writeBuf(ID, reg, &nLen, 1, INST_READ); 
  uint8_t byteRead, i;
  while (_serial.available() > 0) {
    byteRead = _serial.read();
    ESP_LOGI("serial_servo", "%d Read data: %d", i, byteRead);
    if (i == 5) data[0] = byteRead;
    if (i == 6) data[1] = byteRead;
    i++;
  }
}