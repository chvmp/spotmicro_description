#!/usr/bin/env python

import os
import time

import pybullet as p

phisicsClient = p.connect(p.GUI)
p.loadURDF('../urdf/spotmicroai.urdf')
time.sleep(30)
p.disconnect()
