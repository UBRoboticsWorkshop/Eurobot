#ifndef pwmservo_hpp
#define pwmservo_hpp
#include <Arduino.h>

class pwmservo {
public:
    pwmservo(uint8_t pinIN, uint8_t ledCH, float offset = 90, uint16_t lowerPWM = 900, uint16_t upperPWM = 8500, uint16_t lowerAngle = 0, uint16_t upperAngle = 180) {
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

    void moveTo(float angle);
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
