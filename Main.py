#!/usr/bin/env python

import numpy as np
import cv_functions as cvfun


class RobotArm:
    def __init__(self):
        print("Initializing robot arm.")
        self.q = np.zeros([7, 1])
        self.color = 0  # red = 1, green = 2, blue = 3, No targets = 4

        #Not sure we will need all of these. Maybe just kp/ki
        self.kp = 0.0
        self.kd = 0.0
        self.ki = 0.0

    def inv_kine(self, x, y, z):
        print("Calculating inverse kinematics.")
        self.q = np.zeros([7, 1])
        return self.q

    def acquire_targ(self):
        print("Acquiring target.")
        
        # Probably have some while loop that iterates through inv_kine using until it reaches the target.
        #while dist > min:
        #get image here
        #convert to hsv and threshold
        #filter the noise from the thresholding
        #find tomatos
        # get color and target
        #if color == 4: break
        #getDistToCenter
        #if dist < threshold : return color of ball
        #visual servoing on the distance kp * dist + ki * integral of error.
        #get new desired position
        #inverse kinematics
        #command baxter to new position with desired orientation
        self.color = 0
