#ifndef kinematics_hpp
#define kinematics_hpp
#include <Arduino.h>

class kinematics{
  public:
    kinematics(float L0, float L1, float L2, float L3){
      _L0 = L0;
      _L1 = L1;
      _L2 = L2;
      _L3 = L3;

      _Lsum = _L0 + _L1;
      _Ldiff = abs(_L0 - _L1);
    };

    struct Position {
      float Joint0, Joint1, Joint2, Joint3, Joint4;
    } Position_;

    struct ForwardKinematicsOutput {
      float X, Y, Z;
    } ForwardKinematics_, transformed_;

    bool InverseKinematics(float targetX, float targetY, float targetZ, float endeffectorAngle, bool rightly);
    void ForwardKinematics(float Joint0, float Joint1, float Joint2, float Joint3, float Joint4);
    void CoordinateTrans(float originX, float originY, float originZ, float X, float Y, float Z);

  private:
    float _L0, _L1, _L2, _L3;
    float _Lsum;
    float _Ldiff;
    float _Alpha, _Beta, _Gamma;
    //float _Theta;
};

#endif
