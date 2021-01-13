#!/usr/bin/env python

from matrix import get_rotation_matrix, get_transform_matrix


class Kinematics(object):
    def __init__(self, link_list, join_list, link_const):
        self.link_list = link_list
        self.joint_list = joint_list
        self.const_list = const_list

    def forward_kinematics(self, joint_angles):
        for angle in joint_angles:
            for



class Spot(Kinematics):
    def __init__(self, *args, **kwargs):
        super(Spot, self).__init__(*args, **kwargs)
        self.link_list = load_ur
