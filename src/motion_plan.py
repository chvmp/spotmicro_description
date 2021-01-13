#!/usr/bin/env python

import math

from kinematics import Kinematics


class MotionPlan(Kinematics):
    def __init__(self):
        super(MotionPlan, self).__init__()

    def move_path(self, positions, speed):
        """Calculate move path and returns joints angles

        Args:
        positions (np.ndarray)
        speed (float)

        Returns:
        joints (np.ndarray)
        """
        for pos in positions:
            
