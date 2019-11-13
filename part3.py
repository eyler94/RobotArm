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
    limb.set_joint_position_speed(0.8)
    control_rate = rospy.Rate(500)
    

    raw_input("Move to your first desired position and push enter ...")
    end_joint_command = limb.get_joint_angles()

    raw_input("Move to your second desired position and push enter ...")
    start_joint_command = limb.get_joint_angles()

    #######Can Modify me
    time_to_wait = 5
    #######

    number_of_trials = 10


    for i in xrange(number_of_trials):

        start = time.time()
        q = []
        q_dot = []
        t = []

        step = 1
        while step < time_to_wait*500:
            limb.set_joint_positions_mod(end_joint_command)
            q.append(limb.get_joint_angles())
            q_dot.append(limb.get_joint_velocities())
            t.append(time.time()-start)
            control_rate.sleep()
            step = step + 1


        step = 1
        while step < time_to_wait*500:
            limb.set_joint_positions_mod(start_joint_command)
            q.append(limb.get_joint_angles())
            q_dot.append(limb.get_joint_velocities())
            t.append(time.time()-start)
            control_rate.sleep()
            step = step + 1

        data = {"q":q, "q_dot":q_dot, "t":t}
        sio.savemat('/home/radlab/Desktop/part3_trial'+str(i).zfill(2), data)
        print "Completed trial ",str(i+1)
