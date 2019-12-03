#!/usr/bin/env python

import rospy
import tf
import numpy as np
from rad_baxter_limb import RadBaxterLimb
import baxter_interface
import baxter_left_kinematics as blk
import baxter_right_kinematics as brk
import cv_functions as cvfun
import subprocess

class RobotArm:
    def __init__(self):
        print("Initializing robot arm.")
        self.q = np.zeros([7, 1])
        self.color = 4  # red = 1, green = 2, blue = 3, No targets = 4

        #Not sure we will need all of these. Maybe just kp/ki
        self.kp = 0.0
        self.kd = 0.0
        self.ki = 0.0
        self.limb = RadBaxterLimb('right')

        self.control_rate = rospy.Rate(500)

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

    def bash_start_stuff(self):
        Hookup = "cd ~/baxter_ws; ./baxter.sh; cd -"
        # Hookup = "cd ~; ls; cd -"
        process = subprocess.Popen(Hookup, shell=True)

        Untuck_N_Source = "uta; source ~/Desktop/robotics_ws/devel/setup.bash --extend; cd ~/Desktop/robotics_ws/src/rad_baxter_limb/src/rad_baxter_limb"
        process = subprocess.Popen(Untuck_N_Source, shell=True)

    def bash_end_stuff(self):
        process = subprocess.Popen("ta", shell=True)

    def init_spots(self):
        raw_input("Move to the safe position and press enter.")
        self.safe_pos = self.limb.get_joint_angles()
        raw_input("Move to the bucket 1 position and press enter.")
        self.bucket1_pos = self.limb.get_joint_angles()
        raw_input("Move to the bucket 2 position and press enter.")
        self.bucket2_pos = self.limb.get_joint_angles()
        raw_input("Move to the bucket 3 position and press enter.")
        self.bucket3_pos = self.limb.get_joint_angles()

    def init_gripper_right(self):
        self.right_gripper = baxter_interface.Gripper('right')
        self.right_gripper.calibrate()
        self.right_gripper.close()
        self.right_gripper.open()

    def init_gripper_left(self):
        self.left_gripper = baxter_interface.Gripper('left')
        self.left_gripper.calibrate()
        self.left_gripper.close()
        self.left_gripper.open()

    def movin(self, target):
        step = 1
        while step < 2500:
            limb.set_joint_positions_mod(target)
            self.control_rate.sleep()
            step = step + 1

    def move_Bucket(self):
        if self.color==1:
            self.movin(self.bucket1_pos)
        elif self.color==2:
            self.movin(self.bucket2_pos)
        elif self.color==3:
            self.movin(self.bucket3_pos)
        else:
            print("Error, no target color assigned.")


    def main(self):
        bash_start_stuff()

        #Initalize safe spot, buckets 1-3, and calibrate gripper
        self.init_spots()
        self.init_gripper_right()

        # Move to safe spot
        self.movin(self.safe_pos)

        # Open Gripper
        self.right_gripper.open()

        # Acquire/Move to target (set self.color = #)

        # Close Gripper
        self.right_gripper.close()

        # Move to bucket #
        self.move_Bucket()

        # Open Gripper
        self.right_gripper.open()

        # Move to safe spot
        self.movin(self.safe_pos)

        # Check for another ball / run a set number of times and increment.

        # Finish up and tuck the arms
        bash_end_stuff()


if __name__ == '__main__':
    rospy.init_node('BMM_Node')
    RbtArm = RobotArm()
    RbtArm.main()
