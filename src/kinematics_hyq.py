#!/usr/bin/env python

import math

import numpy as np

import cfg.hyq as constants


class BaseKinematic(object):

    def __init__(self):
        super(BaseKinematic, self).__init__()

    def forward_kinematics(self, joint1, joint2, joint3):

        T01 = self.TF_matrix(joint1, 0, constants.hip_leg, math.pi / 2)
        T12 = self.TF_matrix(joint2, 0, constants.upper_leg, 0)
        T23 = self.TF_matrix(joint3, 0, constants.lower_leg, 0)
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)

        return T03[0][-1], T03[1][-1], T03[2][-1]

    def forward_kinematics_test(self, joint1, joint2, joint3):
        x = (constants.lower_leg * math.cos(joint1) * math.cos(joint2) * math.cos(joint3)) - \
            (constants.lower_leg * math.cos(joint1) * math.sin(joint2) * math.sin(joint3)) + \
            (constants.upper_leg * math.cos(joint1) * math.cos(joint2)) + \
            (constants.hip_leg * math.cos(joint1))

        y = (constants.lower_leg * math.sin(joint1) * math.cos(joint2) * math.sin(joint3)) - \
            (constants.lower_leg * math.sin(joint1) * math.sin(joint2) * math.sin(joint3)) + \
            (constants.upper_leg * math.sin(joint1) * math.cos(joint2)) + \
            (constants.hip_leg * math.sin(joint1))

        z = (constants.lower_leg * math.sin(joint2) * math.cos(joint3)) + \
            (constants.lower_leg * math.cos(joint2) * math.sin(joint3)) + \
            (constants.upper_leg * math.sin(joint2))
        return x, y, z

    def TF_matrix(self, joint, d, a, alpha):
        TF = np.array([
            [math.cos(joint), -math.sin(joint)*math.cos(alpha),
             math.sin(joint)*math.sin(alpha), a * math.cos(joint)],
            [math.sin(joint), math.cos(joint)*math.cos(alpha), -
             math.sin(alpha)*math.cos(joint), math.sin(joint) * a],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0,  0,  0,  1]
        ])

        return TF

    def inverse_kinematics(self, x, y, z, revers):
        alpha_1 = math.atan(y/x)

        k1 = x/math.cos(alpha_1) - constants.hip_leg
        k2 = z
        temp = (k1**2 + k2**2 - constants.lower_leg**2 - constants.upper_leg **
                2) / (2 * constants.upper_leg * constants.lower_leg)
        if temp > 1:
            temp = 1
        elif temp < -1:
            temp = -1
        if revers:
            alpha_3 = -math.acos(temp)
        else:
            alpha_3 = math.acos(temp)

        k3 = constants.lower_leg * np.cos(alpha_3) + constants.upper_leg
        k4 = constants.lower_leg * np.sin(alpha_3)
        alpha_2 = math.atan2(k2, k1) - math.atan2(k4, k3)

        return alpha_1, alpha_2, alpha_3

    def format(self, f, n):
        if round(f) == f:
            m = len(str(f))-1-n
            if f/(10**m) == 0.0:
                return f
            else:
                return float(int(f)/(10**m)*(10**m))
        return round(f, n - len(str(int(f)))) if len(str(f)) > n+1 else f

    def forward_kinematics_lf(self, joint1, joint2, joint3):
        z, y, x = self.forward_kinematics(joint1, joint2, joint3)

        x = constants.lf_shoulder[0] - x
        y = constants.lf_shoulder[1] - y
        z = constants.lf_shoulder[2] - z
        return x, y, z

    def inverse_kinematics_lf(self, x, y, z):
        x = constants.lf_shoulder[0] - x
        y = constants.lf_shoulder[1] - y
        z = constants.lf_shoulder[2] - z

        joint1, joint2, joint3 = self.inverse_kinematics(z, y, x, True)
        return joint1, joint2, joint3

    def forward_kinematics_lh(self, joint1, joint2, joint3):
        z, y, x = self.forward_kinematics(joint1, joint2, joint3)

        x = constants.lh_shoulder[0] - x
        y = constants.lh_shoulder[1] - y
        z = constants.lh_shoulder[2] - z
        return x, y, z

    def inverse_kinematics_lh(self, x, y, z):
        x = constants.lh_shoulder[0] - x
        y = constants.lh_shoulder[1] - y
        z = constants.lh_shoulder[2] - z

        joint1, joint2, joint3 = self.inverse_kinematics(z, y, x, False)
        return joint1, joint2, joint3

    def forward_kinematics_rf(self, joint1, joint2, joint3):
        z, y, x = self.forward_kinematics(joint1, joint2, joint3)

        x = constants.rf_shoulder[0] - x
        y = constants.rf_shoulder[1] + y
        z = constants.rf_shoulder[2] - z
        return x, y, z

    def inverse_kinematics_rf(self, x, y, z):
        x = constants.rf_shoulder[0] - x
        y = -constants.rf_shoulder[1] + y
        z = constants.rf_shoulder[2] - z

        joint1, joint2, joint3 = self.inverse_kinematics(z, y, x, True)
        return joint1, joint2, joint3

    def forward_kinematics_rh(self, joint1, joint2, joint3):
        z, y, x = self.forward_kinematics(joint1, joint2, joint3)

        x = constants.rh_shoulder[0] - x
        y = constants.rh_shoulder[1] + y
        z = constants.rh_shoulder[2] - z
        return x, y, z

    def inverse_kinematics_rh(self, x, y, z):
        x = constants.rh_shoulder[0] - x
        y = -constants.rh_shoulder[1] + y
        z = constants.rh_shoulder[2] - z
        joint1, joint2, joint3 = self.inverse_kinematics(z, y, x, False)
        return joint1, joint2, joint3

    def calculate_shoulder(self, rpy_angle, h):
        T1_lf = np.array([
            [1, 0, 0],
            [0, math.cos(rpy_angle[0]), math.sin(rpy_angle[0])],
            [0, -math.sin(rpy_angle[0]), math.cos(rpy_angle[0])]])

        T2_lf = np.array([
            [math.cos(rpy_angle[1]), 0, -math.sin(rpy_angle[1])],
            [0, 1, 0],
            [math.sin(rpy_angle[1]), 0, math.cos(rpy_angle[1])]])

        # print np.dot(constants.lf_init_position, T1)

        lf_new_position = np.dot(
            [constants.lf_init_position[0], constants.lf_init_position[1],
                constants.lf_init_position[2] - h],
            np.dot(T1_lf, T2_lf))
        lh_new_position = np.dot(
            [constants.lh_init_position[0], constants.lh_init_position[1],
                constants.lh_init_position[2] - h],
            np.dot(T1_lf, T2_lf))
        rf_new_position = np.dot(
            [constants.rf_init_position[0], constants.rf_init_position[1],
                constants.rf_init_position[2] - h],
            np.dot(T1_lf, T2_lf))
        rh_new_position = np.dot(
            [constants.rh_init_position[0], constants.rh_init_position[1],
                constants.rh_init_position[2] - h],
            np.dot(T1_lf, T2_lf))

        return lf_new_position, lh_new_position, rf_new_position, rh_new_position

    def print_FK(self, joints):
        x, y, z = self.forward_kinematics_lf(joints[0], joints[1], joints[2])
        print 'lf-joint', x - \
            constants.lf_init_position[0], y - \
            constants.lf_init_position[1], z-constants.lf_init_position[2]

        x, y, z = self.forward_kinematics_lh(joints[3], joints[4], joints[5])

        x, y, z = self.forward_kinematics_rf(joints[6], joints[7], joints[8])

        x, y, z = self.forward_kinematics_rh(joints[9], joints[10], joints[11])
        print 'rh-joint', x - \
            constants.rh_init_position[0], y - \
            constants.rh_init_position[1], z-constants.rh_init_position[2]
