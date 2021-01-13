#!/usr/bin/env python

from math import *

import skrobot
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import *


class Kinematics(Coordinates):

    def __init__(self):
        super(Kinematics, self).__init__()

        link_list = self.link_list
        joint_list = self.joint_list

    def forward_kinematics_base(self):
        return

    def forward_kinematics_leg(self):
        t01 = self.tf_matrix(self.joint1, 0, self.leg1, 0)
        t12 = self.tf_matrix(self.joint2, 0, self.leg2, pi/2)
        t23 = self.tf_matrix(self.joint2, 0, self.leg2, 0)
        t34 = self.tf_matrix(self.joint3, 0, self.leg3, 0)
        t04 = np.dot(np.dot(np.dot(t01, t12), t23), t34)

        return t04

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

    def tf_matrix(self, joint, d, a, alpha):
        tf = np.array([
            [cos(joint), -sin(joint)*cos(alpha), sin(joint)*sin(alpha), a*cos(joint)],
            [sin(joint), cos(joint)*cos(alpha), -sin(alpha)*cos(joint), sin(joint)*a],
            [0, sin(alpha), cos(alpha), d]
            [0, 0, 0, 1]
        ])

        return tf
