from cached_property import cached_property
import os.path as osp

import numpy as np

from skrobot.models.urdf import RobotModelFromURDF
from skrobot.model import RobotModel
from skrobot.coordinates import CascadedCoords


here_dir = osp.dirname(osp.abspath(__file__))


class HYQ(RobotModelFromURDF):
    """HYQ Model with skrobot."""

    def __init__(self, *args, **kwargs):
        super(HYQ, self).__init__(*args, **kwargs)

    @cached_property
    def default_urdf_path(self):
        return osp.join(osp.dirname(here_dir), "urdf/hyq.urdf")

    @cached_property
    def rfront(self):
        pass

    @cached_property
    def lfront(self):
        pass

    @cached_property
    def rrear(self):
        pass

    @cached_property
    def lrear(self):
        pass

    def reset_pose(self):
        pass


if __name__ == "__main__":
    from skrobot.viewers import TrimeshSceneViewer

    robot = HYQ()
    viewer = TrimeshSceneViewer()

    viewer.add(robot)
    robot.reset_pose()

    viewer._init_and_start_app()
