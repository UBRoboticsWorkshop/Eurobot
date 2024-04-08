#ifndef serialservo_hpp
#define serialservo_hpp
#include <Arduino.h>
#include <stdlib.h>

class serialservo{
public:
    serialservo(HardwareSerial &serialport): _serial(serialport){ //Serial2
        _serial.begin(1000000);
    };

    void enabletorque(uint8_t ID, bool enable);
    void moveTo(uint8_t ID, float position, float offset = 180);
    void SetID(uint8_t originID, uint8_t targetID);

private:
    HardwareSerial &_serial;

    void writeBuf(uint8_t ID, uint8_t MemAddr, uint8_t *nDat, uint8_t nLen, uint8_t Fun);
    void writeTest(uint8_t ID, uint8_t reg, uint8_t *data, uint8_t nLen);
    void write16bit(uint8_t ID, uint8_t reg, uint16_t val);
    void write8bit(uint8_t ID, uint8_t reg, uint8_t val);
    void readTest(uint8_t ID, uint8_t reg, uint8_t *data, uint8_t nLen);
};

#endif
