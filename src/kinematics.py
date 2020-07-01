#!/usr/bin/env python

from math import *

import skrobot
from skrobot.coordinates.math import *


class Kinematics(RobotFromURDF):

    def __init__(self):
        link_list = self.link_list
        joint_list = self.joint_list

    def forward_kinematics(self, joint1, joint2, joint3):
        T01 = self.TF_matrix(self.joint1, 0, self.leg1, 0)
        T12 = self.TF_matrix(self.joint2, 0, self.leg2, pi/2)
        T23 = self.TF_matrix(self.joint2, 0, self.leg2, 0)
        T34 = self.TF_matrix(self.joint3, 0, self.leg3, 0)
        T04 = np.dot(np.dot(np.dot(T01, T12), T23), T34)

        return T04
        
    def inverse_kinematics(self, pos, reverses):
        x, y, z = pos
        D = (x**2 + y**2 - self.leg1**2 + z**2 - self.leg2 **
             2 - self.leg3**2) / (2 * self.leg2 * self.leg3)

        self.theta1 = -atan2(-y, x) - atan2(sqrt(x**2+y**2-self.leg1**2), -self.leg1)
        self.theta2 = atan2(z, sqrt(x**2+y**2-self.leg1**2)) - atan2(self.leg3*sin(self.theta3), self.leg2+self.leg3*cos(self.theta3))
        if revers:
            self.theta3 = atan2(-sqrt(1-D**2), D)
        else:
            self.theta3 = atan2(sqrt(1-D**2), D)

        return self.theta1, self.theta2, self.theta3
