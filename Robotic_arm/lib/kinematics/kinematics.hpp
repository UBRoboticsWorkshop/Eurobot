#include <Arduino.h>

class kinematics{
  public:
    kinematics(float L1, float L2){
      _L1 = L1;
      _L2 = L2;
    };

    struct Position {
      //float Joint0, Joint1, Joint2, Joint3;
      float RightlyJoint0, RightlyJoint1, RightlyJoint2, RightlyJoint3;
      float LeftyJoint0, LeftyJoint1, LeftyJoint2, LeftyJoint3;
      bool Lefty, Rightly;
    } Position_;

    struct ForwardKinematicsOutput {
      float X, Y, Z;
    } ForwardKinematics_, transformed_;

    void InverseKinematics(float targetX, float targetY, float targetZ)
    void ForwardKinematics(float Joint0, float Joint1, float Joint2)
    void CoordinateTrans(float originX, float originY, float originZ, float X, float Y, float Z)

  private:
    float _L1, _L2;
    float _Lsum = _L1 + _L2;
    float _Ldiff = abs(_L1 - _L2);
    float _Alpha, _Beta, _Gamma;
    float _xy;
    //float _Theta;
};