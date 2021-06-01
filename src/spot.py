from cached_property import cached_property
import os.path as osp

from skrobot.models.urdf import RobotModelFromURDF

here_dir = osp.dirname(osp.abspath(__file__))


class Spot(RobotModelFromURDF):
    def __init__(self, *args, **kwargs):
        super(Spot, self).__init__(*args, **kwargs)

    @cached_property
    def default_urdf_path(self):
        return osp.join(osp.dirname(here_dir), "urdf/spotmicroai.urdf")

    def rleg(self):
        pass

    def lleg(self):
        pass

    def rfoot(self):
        pass

    def lfoot(self):
        pass
    

