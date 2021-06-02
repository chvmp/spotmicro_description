from cached_property import cached_property
import os.path as osp

import numpy as np

from skrobot.models.urdf import RobotModelFromURDF
from skrobot.model import RobotModel
from skrobot.coordinates import CascadedCoords


here_dir = osp.dirname(osp.abspath(__file__))


class Spot(RobotModelFromURDF):
    """Spot Model with skrobot

    Ref: https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot

    <links>
        - base_link
        - lidar_link
        - rear_link
        - front_link
        <(front/rear)_(left/right)_...>
        - front_left_shoulder_link
        - front_left_leg_link_cover
        - front_left_leg_link
        - front_left_foot_link
        - front_left_toe_link

    <joints>
        <fixed>
        - base_lidar
        - base_rear
        - base_front
        <(front/rear)_(left/right)_...>
        - front_left_shoulder
        - front_left_leg
        - front_left_foot
    """
    def __init__(self, *args, **kwargs):
        super(Spot, self).__init__(*args, **kwargs)

        self.rfront_end_coords = CascadedCoords(
            parent=self.front_right_toe_link,
            name='rfont_end_coords'
        )
        self.lfront_end_coords = CascadedCoords(
            parent=self.front_left_toe_link,
            name='lfont_end_coords'
        )
        self.rrear_end_coords = CascadedCoords(
            parent=self.rear_right_toe_link,
            name='rrear_end_coords'
        )
        self.lrear_end_coords = CascadedCoords(
            parent=self.rear_left_toe_link,
            name='lrear_end_coords'
        )

        # TODO: add joints limit(max/min)

    @cached_property
    def default_urdf_path(self):
        return osp.join(osp.dirname(here_dir), "urdf/spotmicroai.urdf")

    @cached_property
    def rfront(self):
        rfront_links = [
            self.front_right_shoulder_link, 
            self.front_right_leg_link_cover, 
            self.front_right_leg_link, 
            self.front_right_foot_link, 
            self.front_right_toe_link
        ]
        rfront_joints = []
        for link in rfront_links:
            rfront_joints.append(link.joint)
        r = RobotModel(link_list=rfront_links, joint_list=rfront_joints)
        r.end_coords = self.rfront_end_coords
        return r

    @cached_property
    def lfront(self):
        lfront_links = [
            self.front_left_shoulder_link, 
            self.front_left_leg_link_cover, 
            self.front_left_leg_link, 
            self.front_left_foot_link, 
            self.front_left_toe_link
        ]
        lfront_joints = []
        for link in lfront_links:
            lfront_joints.append(link.joint)
        r = RobotModel(link_list=lfront_links, joint_list=lfront_joints)
        r.end_coords = self.lfront_end_coords
        return r

    @cached_property
    def rrear(self):
        rrear_links = [
            self.rear_right_shoulder_link, 
            self.rear_right_leg_link_cover, 
            self.rear_right_leg_link, 
            self.rear_right_foot_link, 
            self.rear_right_toe_link
        ]
        rrear_joints = []
        for link in rrear_links:
            rrear_joints.append(link.joint)
        r = RobotModel(link_list=rrear_links, joint_list=rrear_joints)
        r.end_coords = self.rrear_end_coords
        return r

    @cached_property
    def lrear(self):
        lrear_links = [
            self.rear_left_shoulder_link, 
            self.rear_left_leg_link_cover, 
            self.rear_left_leg_link, 
            self.rear_left_foot_link, 
            self.rear_left_toe_link
        ]
        lrear_joints = []
        for link in lrear_links:
            lrear_joints.append(link.joint)
        r = RobotModel(link_list=lrear_links, joint_list=lrear_joints)
        r.end_coords = self.lrear_end_coords
        return r

    def reset_pose(self):
        self.front_left_shoulder.joint_angle(np.deg2rad(11.5735))
        self.front_left_leg.joint_angle(np.deg2rad(33.0804))
        self.front_left_foot.joint_angle(np.deg2rad(-100.5692))
        self.front_right_shoulder.joint_angle(np.deg2rad(11.5735))
        self.front_right_leg.joint_angle(np.deg2rad(33.0804))
        self.front_right_foot.joint_angle(np.deg2rad(-100.569))
        self.rear_left_shoulder.joint_angle(np.deg2rad(7.5883))
        self.rear_left_leg.joint_angle(np.deg2rad(-28.7493))
        self.rear_left_foot.joint_angle(np.deg2rad(29.7695))
        self.rear_right_shoulder.joint_angle(np.deg2rad(7.5883))
        self.rear_right_leg.joint_angle(np.deg2rad(-28.7493))
        self.rear_right_foot.joint_angle(np.deg2rad(29.7695))
        return self.angle_vector()


if __name__ == "__main__":
    from skrobot.viewers import TrimeshSceneViewer

    robot = Spot()
    viewer = TrimeshSceneViewer()

    viewer.add(robot)
    robot.reset_pose()

    viewer._init_and_start_app()
