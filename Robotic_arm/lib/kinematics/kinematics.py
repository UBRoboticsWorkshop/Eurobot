from math import acos, atan2, sin, cos, sin
import math.pi as PI



class kinematics:
    def __init__(self, L0, L1, L2, L3, L4):
        self._L0 = L0
        self._L1 = L1
        self._L2 = L2
        self._L3 = L3
        self._L4 = L4

        self._Lsum = L0 + L1
        self._Ldiff = abs(L0 - L1)

    # end effector angle in XY_Z 2D pannel > 0
    def InverseKinematics(self, targetX, targetY, targetZ, endeffectorAngle, Joint4, Lefty = true):
        Joint0, Joint1, Joint2, Joint3, Joint4 = 0.0, 0.0, 0.0, 0.0, Joint4

        # fix end effector direction : unit vector x (L3 + L4)
        targetZJ3 = targetZ + sin(180.0 - endeffectorAngle) * (self._L3 + self._L4)
        x_y = (targetX**2 + targetY**2) ** 2 - cos(180.0 - endeffectorAngle) * (self._L3 + self._L4)# Length in xy plannel
        xy_z = (x_y**2 + targetZJ3**2) ** 2 # Length in xy_z plannel

        if ((xy_z <= _Lsum) and (xy_z >= _Ldiff)):
            _Alpha = acos((xy_z**2 + self._L0**2 - self._L1**2) / (2 * self._L0 * xy_z)) * 180.0 / PI
            _Beta = acos((self._L0**2 + self._L1**2 - xy_z**2) / (2 * self._L0 * self._L1)) * 180.0 / PI
            _Gamma = atan2(targetZJ3, x_y) * 180.0 / PI

            Joint0 = atan2(targetY, targetX) * 180.0 / PI # Joint or end effector does not matter

            RightlyJoint1 = _Gamma - _Alpha
            RightlyJoint2 = 180.0 - _Beta

            LeftyJoint1 = _Gamma + _Alpha
            LeftyJoint2 = _Beta - 180.0

            if (Lefty):
                Joint1 = LeftyJoint1
                Joint2 = LeftyJoint2
            else:
                Joint1 = RightlyJoint1
                Joint2 = RightlyJoint2

        else:
            print("OUT OFF RANGE!")
            #Joint0, Joint1, Joint2, Joint3, Joint4 = InverseKinematics(targetX, targetY, targetZ, endeffectorAngle, Joint4, Lefty) # TODO: change the end effectors angle

        if (endeffectorAngle < 180.0):
            Joint3 = 180.0 + endeffectorAngle - Joint1 - Joint2 + 360.0
        else:
            Joint3 = -180.0 + endeffectorAngle - Joint1 - Joint2 + 360.0

        return Joint0, Joint1, Joint2, Joint3, Joint4

    def ForwardKinematics(self, Joint0, Joint1, Joint2, Joint3, Joint4):
        Z = _L1 * sin(Joint1) + _L2 * sin(Joint1 + Joint2)
        _xy = _L1 * cos(Joint1) + _L2 * cos(Joint1 + Joint2)
        X = _xy * cos(Joint0)
        Y = _xy * sin(Joint0)

        return X, Y, Z

    def CoordinateTrans(self, originX, originY, originZ, X, Y, Z):
        X = X - originX
        Y = Y - originY
        Z = Z - originZ

        return X, Y, Z