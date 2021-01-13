#!/usr/bin/env python

from kinematics import Kinematics
from utils.cfg import parse_cfg


class RobotModel(Kinematics):
    def __init__(self, link_list=None, join_list=None, reverse=None):
        super(RobotModel, self).__init__(link_list, join_list, reverse)

        self.joint_names = []
        for joint, link in zip(self.joint_list, self.link_list):
            self.joint_names.append(joint.name)
            self.__dict__[link.name] = link
            self.__dict__[joint.name] = joint

    def reset_pose(self):
        raise NotImplementedError()

    def parse_cfg(self, cfg):
        """Parse model config.cfg and returns model config as dict"""
        return parse_cfg(cfg)

    def load_urdf(self, urdf):
        return

    def load_urdf_file(self, file_obj):
        return
