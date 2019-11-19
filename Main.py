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
        self.q = np.zeros([7, 1]) # Be best if we use the current configuration. Closer to the answer
        return self.q

    def acquire_targ(self):
        print("Acquiring target.")
        dist = np.array([1e6, 1e6])
        min_dist = 3 #Pixels. May want to change this
        
        # Probably have some while loop that iterates through inv_kine using until it reaches the target.
        # while dist[0] > min_dist and dist[1] > min_dist:
        #     img = getImg() #How do we get this?
        #     hsv = cvfun.convertToHSV(img)
        #     img_b, img_g, img_r = cvfun.extractColors(hsv)
        #     img_b, img_g, img_r = cvfun.filterNoise(img_b, img_g, img_r)
        #     pts_b, pts_g, pts_r = cvfun.findTomatoes(img_b, img_g, img_r)
        #     self.color, target = cvfun.getColorAndTarget(pts_b, pts_g, pts_r)
        #     if self.color == 4: # no targets
        #         break
        #     img_size = img_b.shape
        #     center_pt = np.array(img_size)/2.0
        #     dx, dy = cvfun.getDistToCenter(center_pt, target)
        #     delta_x = dx * kp #Will need to add these to the current position
        #     delta_y = dy * kp 
        #     x = x + delta_x
        #     y = y + delta_y
        #     q = self.inv_kine(x, y, z)
        #     # baxter.setPosition(q)
        return self.color