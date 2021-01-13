#!/usr/bin/env python

import numpy as np


class Kinematics(object):
    """Compute kineamtics for each part, e.g. left-leg
    """

    def __init__(self, link_list, joint_list, reverse, reflect):
        self.link_list = link_list
        self.joint_list = joint_list

        self.reverse = reverse
        self.reflect = reflect

    def forward_kinematics(self, joint_angles):
        """Returns calculated position of end-effector

        Args:
            joint_angles (list): Target joint angles for each joint

        Returns:
            np.ndarray: in shape (3,)
        """
        assert len(joint_angles) == len(self.joint_list), (
            "joint_angles must be same length of joint_list-{}".format(
                len(self.joint_list))
        )

        if isinstance(joint_angles, np.ndarray):
            joint_angles = joint_angles.tolist()

        TF_04 = np.ones((4, 4))
        for joint, angle in zip(self.joint_list, joint_angles):
            TF_04 = np.dot(TF_04, self._calc_TF_matrix(angle, joint))

        pos = self.link_list.shoulder * np.array([1, self.reflect, 1]) + \
            TF_04[:3, -1] * np.array([-1, -self.reflect, -1])
        return pos

    def _calc_TF_matrix(self, angle, joint):
        alpha = 0
        if "hip" in joint.name:
            alpha = np.pi / 2
        tf = self._get_TF_matrix(angle, 0, joint, alpha)
        return tf

    def _get_TF_matrix(self, joint, d, a, alpha):
        """Returns transform matrix
        """
        TF = np.array([
            [np.cos(joint), -np.sin(joint)*np.cos(alpha),
             np.sin(joint)*np.sin(alpha), a * np.cos(joint)],
            [np.sin(joint), np.cos(joint)*np.cos(alpha), -
             np.sin(alpha)*np.cos(joint), np.sin(joint) * a],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0,  0,  0,  1]
        ])
        return TF

    def inverse_kinematics(self, pos, rthre=0.001):
        if isinstance(pos, list):
            pos = np.array(pos)
        pos = self.link_list.shoulder * np.array([1, self.reflect, 1]) + \
            pos * np.array([-1, -self.reflect, -1])
        while True:
            ret = self._inverse_kinematics_loop(pos.tolist())
            if ret == "ik_continue":
                pass
            else:
                break
        return ret

    def _inverse_kinematics_loop(self, tgt_pos, diff_pos, rthre):
        x, y, z = diff_pos
        alpha_1 = np.arctan(y / x)
        k1 = x / np.cos(alpha_1) - self.link_list.hip_leg
        k2 = z
        temp = \
            (k1 ** 2 + k2 ** 2 -
             self.link_list.lower_leg ** 2 -
             self.link_list.upper_leg ** 2) \
            / (2 * self.link_list.upper_leg * self.link_list.lower_leg)

        if temp > 1:
            temp = 1
        elif temp < -1:
            temp = -1

        if self.reverse:
            alpha_3 = -np.arccos(temp)
        else:
            alpha_3 = np.arccos(temp)

        k3 = self.link_list.lower_leg * \
            np.cos(alpha_3) + self.link_list.upper_leg
        k4 = self.link_list.lower_leg * np.sin(alpha_3)
        alpha_2 = np.arctan2(k2, k1) - np.arctan2(k4, k3)

        ret = self._eval_finish(tgt_pos, alpha_1, alpha_2, alpha_3, rthre)
        return ret

    def _eval_angle(self, alpha_1, alpha_2, alpha_3):
        ret = True
        if (self.joint_list.bound.haa[0] < alpha_1 or
            self.joint_list.bound.haa[1] > alpha_1 or
            self.joint_list.bound.hfe[0] < alpha_2 or
            self.joint_list.bound.hfe[1] > alpha_2 or
            self.joint_list.bound.kfe[0] < alpha_3 or
                self.joint_list.bound.kfe[1] > alpha_3):
            ret = False
        return ret

    def _eval_finish(self, tgt_pos, alpha_1, alpha_2, alpha_3, rthre):
        ret = self._eval_angle(alpha_1, alpha_2, alpha_3)
        if ret:
            res_pos = self.forward_kinematics([alpha_1, alpha_2, alpha_3])
            if rthre > np.linalg.norm(tgt_pos - res_pos):
                ret = "ik_continue"
        return ret
