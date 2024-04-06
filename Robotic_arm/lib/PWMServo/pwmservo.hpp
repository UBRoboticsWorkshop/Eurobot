#ifndef pwmservo_hpp
#define pwmservo_hpp
#include <Arduino.h>

class pwmservo {
public:
    pwmservo(uint8_t pinIN, uint8_t ledCH, uint16_t lowerPWM = 10, uint16_t upperPWM = 65535, uint16_t lowerAngle = 0, uint16_t upperAngle = 180, float offset = 90) {
        _pinIN = pinIN; 
        _ledCH = ledCH;         // 0 - 15 are availible by default

        _lowerPWM = lowerPWM;
        _upperPWM = upperPWM;

        _lowerAngle = lowerAngle;
        _upperAngle = upperAngle;

        _offset = offset;

        pinMode(_pinIN, OUTPUT);
        ledcSetup(_ledCH, 50, 16);
        ledcAttachPin(_pinIN, _ledCH);
    };

    void servoSetAngle(float angle);
    void servoGoto(uint16_t angle);
    void servoStop();

private:
    uint8_t _pinIN;
    uint8_t _ledCH;
    uint16_t _lowerPWM, _upperPWM;
    uint16_t _lowerAngle, _upperAngle;
    float _offset;
};

#endif
