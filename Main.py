#!/usr/bin/env python

import numpy as np


class RobotArm:
    def __init__(self):
        print("Initializing robot arm.")
        self.q = np.zeros([7, 1])
        self.color = 0  # 0=blue, 1=red, 2=green

    def inv_kine(self, x, y, z):
        print("Calculating inverse kinematics.")
        self.q = np.zeros([7, 1])
        return self.q

    def acquire_targ(self):
        print("Acquiring target.")
        # Probably have some while loop that iterates through inv_kine using until it reaches the target.
        self.color = 0
