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
        self.limb = RadBaxterLimb('right')

    def inv_kine(self, x, y, z):
        print("Calculating inverse kinematics.")

        # Calculate the necessary orientation, it is constant, so we just need to know what it is.
        # TODO: Put the baxter in the orientation desired and input that into here.
        des_or = np.array([0.0, 0.0, 0.0])

        # Calculate the errors
        curr_pos = limb.get_kdl_forward_position_kinematics()
        # TODO: Get current orientation into axis angle state based on what is coming out from previous function
        curr_pos = curr_pos[0:3]
        err_pos = np.array([x, y, z]) - curr_pos
        err_or = des_or - curr_or

        # Calculate the errors
        error = np.array([err_pos[0],err_pos[1],err_pos[2],err_or[0],err_or[1],err_or[2]])
        k = np.diag((self.kp,self.kp,self.kp,self.ki,self.ki,self.ki))

        e = np.matmul(k,error)

        Jt = self.limb.get_kdl_jacobian_transpose()
        # Can make one row of the Jacobian zero if we don't care about one axis of orientation, or make e zero for that joint

        self.q = self.limb.get_joint_angles() + np.matmul(Jt,e)
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