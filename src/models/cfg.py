#!/usr/bin/env python

from ..robot_model import RobotModel


class RobotModelFromCfg(RobotModel):
    def __init__(self, cfg=None):
        super(RobotModelFromCfg, self).__init__()

        if cfg:
            self.parsed = self.load_cfg(cfg)
        else:
            self.parsed = self.load_cfg(self.default_cfg_path)

    @property
    def default_cfg_path(self):
        raise NotImplementedError
