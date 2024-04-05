#include "./kinematics.hpp"
static const char* TAG = "kinematics";
#include <Arduino.h>



void kinematics::InverseKinematics(float targetX, float targetY, float targetZ, float endeffectorAngle) {
  //fix end effector direction
  float targetZJ3 = targetZ + sin(180.0f - endeffectorAngle) * (_L3 + _L4);

  float x_y = sqrt(targetX * targetX + targetY * targetY) - cos(180.0f - endeffectorAngle) * (_L3 + _L4);
  float xy_z = sqrt(x_y * x_y + targetZJ3 * targetZJ3);

  if (xy_z <= _Lsum && xy_z >= _Ldiff) {
    _Alpha = acos((xy_z*xy_z + _L0*_L0 - _L1*_L1) / (2 * _L0 * xy_z)) * 180.0f / PI;
    _Beta = acos((_L0*_L0 + _L1*_L1 - xy_z*xy_z) / (2 * _L0 * _L1)) * 180.0f / PI;
    _Gamma = atan2(targetZJ3, x_y) * 180.0f / PI;

    Position_.Joint0 = atan2(targetY, targetX) * 180.0f / PI;

    Position_.RightlyJoint1 = _Gamma - _Alpha;
    Position_.LeftyJoint1 = _Gamma + _Alpha;

    Position_.RightlyJoint2 = 180.0f - _Beta;
    Position_.LeftyJoint2 = _Beta - 180.0f;

    if (endeffectorAngle < 180.0f){
      Position_.RightlyJoint3 = 180.0f + endeffectorAngle - Position_.RightlyJoint1 - Position_.RightlyJoint2 + 360.0f;
      Position_.LeftyJoint3 = 180.0f + endeffectorAngle - Position_.LeftyJoint1 - Position_.LeftyJoint2 + 360.0f;
    } else {
      Position_.RightlyJoint3 = -180.0f + endeffectorAngle - Position_.RightlyJoint1 - Position_.RightlyJoint2 + 360.0f;
      Position_.LeftyJoint3 = -180.0f + endeffectorAngle - Position_.LeftyJoint1 - Position_.LeftyJoint2 + 360.0f;
    }
  } else {
    ESP_LOGW(TAG, "OUT OF RANGE!");
  }
};

void kinematics::ForwardKinematics(float Joint0, float Joint1, float Joint2) {
  ForwardKinematics_.Z = _L1 * sin(Joint1) + _L2 * sin(Joint1 + Joint2);
  float _xy = _L1 * cos(Joint1) + _L2 * cos(Joint1 + Joint2);
  ForwardKinematics_.X = _xy * cos(Joint0);
  ForwardKinematics_.Y = _xy * sin(Joint0);
};

void kinematics::CoordinateTrans(float originX, float originY, float originZ, float X, float Y, float Z) {
  transformed_.X = X - originX;
  transformed_.Y = Y - originY;
  transformed_.Z = Z - originZ;
};
