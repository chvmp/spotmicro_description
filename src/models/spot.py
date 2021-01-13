#!/usr/bin/env python

from cached_property import cached_property
import os.path as osp

from cfg import RobotModelFromCfg
from ..robot_model import RobotModel


class Spot(RobotModelFromCfg):
    def __init__(self, *args, **kwargs):
        super(Spot, self).__init__(*args, **kwargs)

        for name in self.parsed:
            self.link_list[name] = self.parsed[name]["length"]
            self.joint_list[name] = self.parsed

    @cached_property
    def default_cfg_path(self):
        return osp.join(osp.dirname(osp.abspath(__file__)), "cfg/spot.cfg")

    @cached_property
    def lfoot(self):
        link_list = self.link_list["lf"]
        joint_list = self.joint_list["lf"]
        r = RobotModel(link_list, joint_list, True)
        return r

    @cached_property
    def lhip(self):
        link_list = self.link_list["lh"]
        joint_list = self.joint_list["lh"]
        r = RobotModel(link_list, joint_list, False)
        return r

    @cached_property
    def rfoot(self):
        link_list = self.link_list["rf"]
        joint_list = self.joint_list["rf"]
        r = RobotModel(link_list, joint_list, True)
        return r

    @cached_property
    def rhip(self):
        link_list = self.link_list["rh"]
        joint_list = self.joint_list["rh"]
        r = RobotModel(link_list, joint_list, False)
        return r

    def reset_pose(self):
        self.set_joint_angle()
