#system level imports
import sys, os
from collections import deque
import numpy as np
import scipy.io as sio
#!/usr/local/bin/python

from copy import deepcopy
from threading import RLock, Timer
import time
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('me_537_lab')
    limb = RadBaxterLimb('right')

    num_trials = 4

    for i in xrange(0,num_trials):

        raw_input("Go to position "+str(i+1)+" using the wrist button on baxter. Then press Enter...")
        pose = limb.get_kdl_forward_position_kinematics()
        R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
        position = pose[0:3]

        print "\njoint angles are: \n", limb.get_joint_angles()
        print "\nend effector position is: \n", position
        print "\nend effector orientation is: \n", R, "\n\n\n"


    # #modify this as you want
    # joint_command = [0, 0, 0, 0, 0, 0, 0]

    # while not rospy.is_shutdown():
    #     control_rate = rospy.Rate(500)
    #     limb.set_joint_positions_mod(joint_command)
    #     control_rate.sleep()
