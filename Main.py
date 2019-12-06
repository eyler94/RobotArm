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
import shlex
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from copy import deepcopy
import cv2
import argparse
from time import sleep

class RobotArm:
    def __init__(self):
        print("Initializing robot arm.")
        self.q = np.zeros([7, 1])
        self.color = 0  # red = 1, green = 2, blue = 3, No targets = 4

        #Not sure we will need all of these. Maybe just kp/ki
        self.kp = 0.00005
        self.kd = 0.0
        self.ki = 0.0
        self.limb = RadBaxterLimb('right')

        #inverse kinematic gains
        # self.k1 = 20 
        # self.k2 = 01
        self.k1 = 0.1 
        self.k2 = 0.001


        #image subscriber
        self.img_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.imgCallback)
        self.img = None
        self.CvImg = CvBridge()

        self.control_rate = rospy.Rate(500)
        self.des_or = np.array(3)
    
    def imgCallback(self, data):
        # print("Converting Image")
        try:
            self.img = self.CvImg.imgmsg_to_cv2(data, "bgr8")
            self.img = self.img[100:700, 200:1000,:]
        except CvBridgeError as e:
            print(e)
            
    def inv_kine(self, x, y, z):
        print("Calculating inverse kinematics.")
        eps = .001
        i = 0

        # Calculate the necessary orientation, it is constant, so we just need to know what it is.
        
        # des_or = np.array([ 0.51176522,  3.07447603, -0.01423531])
        des_or = self.des_or

        while i < 1000: #max number of iterations so it doesn't run forever
            # Calculate the errors
            curr_pos = self.limb.get_kdl_forward_position_kinematics()
            q = curr_pos[3:]
            R = tf.transformations.quaternion_matrix(q)
            theta = np.arccos((np.trace(R) - 1)/2.0)
            temp = np.array([R[2,1] - R[1,2], R[0,2]-R[2,0], R[1,0] - R[0,1]])
            k = temp/(2 * np.sin(theta))
            curr_or = k * theta
            
            curr_pos = curr_pos[0:3]
            err_pos = np.array([x, y, z]) - curr_pos
            # print(err_pos)
            err_or = -des_or + curr_or
            # print(err_or)

            # Calculate the errors
            error = np.array([err_pos[0],err_pos[1],err_pos[2],err_or[0],err_or[1],err_or[2]])
            k = np.diag((self.k1,self.k1,self.k1,self.k2,self.k2,self.k2))

            e = np.dot(k,error)
            # print(e)
            if np.linalg.norm(error) < eps:
                break

            Jt = self.limb.get_kdl_jacobian_transpose()
            # Can make one row of the Jacobian zero if we don't care about one axis of orientation, or make e zero for that joint

            self.q = self.limb.get_joint_angles() + np.dot(Jt,e)
            i+=1
        return np.asarray(self.q).reshape(7)

    def acquire_targ(self):
        print("Acquiring target.")
        # dist = np.array([1e6, 1e6])
        dx = 1e6
        dy = 1e6
        min_dist = 10 #Pixels. May want to change this
        curr_pos = self.limb.get_kdl_forward_position_kinematics()
        x = curr_pos.item(0)
        y = curr_pos.item(1)
        z = curr_pos.item(2)

        # Probably have some while loop that iterates through inv_kine using until it reaches the target.
        while (np.abs(dx) > min_dist and np.abs(dy) > min_dist):
            img = deepcopy(self.img)
            hsv = cvfun.convertToHSV(img)
            img_b, img_g, img_r = cvfun.extractColors(hsv)
            img_b, img_g, img_r = cvfun.filterNoise(img_b, img_g, img_r)
            # cv2.imshow('blue', img_b)
            # cv2.imshow('red', img_r)
            # cv2.imshow('green', img_g)
            # cv2.imshow('orig', img)
            # cv2.waitKey(0)

            pts_b, pts_g, pts_r = cvfun.findTomatoes(img_b, img_g, img_r)

            self.color, target = cvfun.getColorAndTarget(pts_b, pts_g, pts_r)
            print('Color: ', self.color)
            if self.color == 4: # no targets
                break

            img_size = img_b.shape
            center_pt = np.array(img_size)/2.0
            dx, dy = cvfun.getDistToCenter(center_pt, target)
            delta_x = dx * self.kp 
            delta_y = dy * self.kp
            print('Dx: ', delta_x)
            print('Dy: ', delta_y)
            x = x - delta_x # + or -
            y = y - delta_y
            q = self.inv_kine(x, y, z)
            print('q', q)
            print('shape', q.shape)
            self.limb.set_joint_positions_mod(q)
        return self.color, x, y
    
    def descend(self, x, y, z_ball):
        print("descending")
        q_des = self.inv_kine(x, y, z_ball)
        print('qdes:,', q_des)
        # raw_input("Hit enter")
        self.movin(q_des) #may need to repeat this or shove this function in a while loop and do smaller steps

    def bash_start_stuff(self):
        Hookup = "cd ~/baxter_ws; ./baxter.sh; cd -"
        # Hookup = "cd ~; ls; cd -"
        process = subprocess.Popen(Hookup, shell=True)

        Untuck_N_Source = shlex.split("uta; source ~/Desktop/robotics_ws/devel/setup.bash --extend; cd ~/Desktop/robotics_ws/src/rad_baxter_limb/src/rad_baxter_limb")
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
        print("collecting z height.")
        self.movin(self.safe_pos)
        raw_input("Move to table height and press enter.")
        position = self.limb.get_kdl_forward_position_kinematics()
        self.z_ball = position[2]*np.ones([7])




    def init_gripper_right(self):
        self.right_gripper = baxter_interface.Gripper('right')
        self.right_gripper.calibrate()
        # self.right_gripper.close()
        # self.right_gripper.open()

    def init_gripper_left(self):
        self.left_gripper = baxter_interface.Gripper('left')
        self.left_gripper.calibrate()
        self.left_gripper.close()
        self.left_gripper.open()

    def movin(self, target):
        # print('Target', target.shape)
        step = 1
        while step < 2500:
            self.limb.set_joint_positions_mod(target)
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

    def main(self, args):
        # self.bash_start_stuff()
        # cd ~/baxter_ws; ./baxter.sh; cd -; uta; source ~/Desktop/robotics_ws/devel/setup.bash --extend; cd ~/Desktop/robotics_ws/src/rad_baxter_limb/src/rad_baxter_limb

        #Initalize safe spot, buckets 1-3, and calibrate gripper
        if not args.get("spots", False):
            print("No location file provided.")
            self.init_spots()
            spots = np.array([self.safe_pos, self.bucket1_pos, self.bucket2_pos, self.bucket3_pos, self.z_ball])
            np.save("spots.npy", spots)
        else:
            print("Loading file.")
            spots = np.load(args["spots"])
            self.safe_pos = spots[0]
            self.bucket1_pos = spots[1]
            self.bucket2_pos = spots[2]
            self.bucket3_pos = spots[3]
            self.z_ball = spots[4]

        self.init_gripper_right()

        # Move to safe spot
        self.movin(self.safe_pos)
        pos = self.limb.get_kdl_forward_position_kinematics()
        q = pos[3:]
        R = tf.transformations.quaternion_matrix(q)
        theta = np.arccos((np.trace(R) - 1)/2.0)
        temp = np.array([R[2,1] - R[1,2], R[0,2]-R[2,0], R[1,0] - R[0,1]])
        k = temp/(2 * np.sin(theta))
        self.des_or = k * theta


        # Open Gripper
        self.right_gripper.open()

        while not self.color==4:
            # Acquire/Move to target (set self.color = #)
            self.color, x, y = self.acquire_targ()
            print("self.z_ball", self.z_ball)
            self.descend(x, y, self.z_ball[0])

            # Close Gripper
            self.right_gripper.close()
            sleep(0.5)
    
            # Move to bucket
            self.move_Bucket()

            # Open Gripper
            self.right_gripper.open()
            sleep(0.5)
    
            # Move to safe spot
            self.movin(self.safe_pos)

            # Check for another ball / run a set number of times and increment.

        # Finish up and tuck the arms
        # self.bash_end_stuff()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--spots",
                    help="Add optional spots file.")
    args = vars(ap.parse_args())

    rospy.init_node('BMM_Node')
    RbtArm = RobotArm()
    RbtArm.main(args)
