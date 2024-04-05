class pwmservo {
public:
    pwmservo(uint8_t pinIN, uint8_t ledCH, uint16_t lowerPWM = 10, uint16_t upperPWM = 65535, float lowerAngle = 0, float upperAngle = 180) {
        _pinIN = pinIN; 
        _ledCH = ledCH;         // 0 - 15 are availible by default

        _lowerPWM = lowerPWM;
        _upperPWM = upperPWM;
        _PWMDifference = upperPWM - lowerPWM;

        _lowerAngle = lowerAngle;
        _upperAngle = upperAngle;
        _angleDifference = upperAngle - lowerAngle;

        pinMode(_pinIN, OUTPUT);
        ledcSetup(_ledCH, 50, 16);
        ledcAttachPin(_pinIN, _ledCH);
    };

    void servoGo(float angele); 
    void servoStop();

private:
    uint8_t _pinIN;
    uint8_t _ledCH;
    uint16_t _lowerPWM, _upperPWM, _PWMDifference;
    float _lowerAngle, _upperAngle, _angleDifference;
};