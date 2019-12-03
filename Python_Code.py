import rospy
import tf
import numpy as np
from rad_baxter_limb import RadBaxterLimb
import baxter_interface
import baxter_left_kinematics as blk
import baxter_right_kinematics as brk

#From From Lab 1 Start_robot
rospy.init_node('Seth_Freeman_Node')
r_limb = RadBaxterLimb('right')
l_limb = RadBaxterLimb('left')

#From Lab 1 Part 1, fk for whole arm, encorperates gripper.
pose = r_limb.get_kdl_forward_position_kinematics()
R = tf.transformations.quaternion_matrix(pose[3:])[0:3,0:3]
position = pose[0:3]

#From Lab 1 Part 2, move arm to certain joint angles
joint_command_start = np.array([0, 0, 0, 0, 0, 0, 0])
control_rate = rospy.Rate(500)
r_limb.set_joint_position_speed(0.5)
step = 1
while step < 2500:
    r_limb.set_joint_positions_mod(joint_command_start)
    control_rate.sleep()
    step = step + 1

# From My Past Code
right_gripper = baxter_interface.Gripper('right')
right_gripper.calibrate()
right_gripper.close()
right_gripper.open()

#Stuff rad_baxter_limb can do
r_limb.get_joint_angles()
r_limb.get_kdl_jacobian()
r_limb.get_kdl_jacobian_transpose()
r_limb.get_kdl_jacobian_pseudo_inverse()
r_limb.get_kdl_jacobian_transpose()
r_limb.kin_kdl.inverse_kinematrics(np.array[0.3, 0.3, 0.3]))

# joint by joint forward kinematics and jacobians. This stuff is a bit complicated so it will get a little bit more explination
# this code replaces some of the functionality we would expect from the matlab toolbox, but offers it in python.
# it uses sympy to calculate THEORETICAL forward kinematics and jacobians. That is not ideal, but it is the only way we can get those values in python
# at certain joints rather than just at the end. It is important to know that because it is theoretical it does not incorperate the gripper.
# So the values you get from the rad_baxter_limb code will be different by that gripper length.
# You pass in a python array (not a numpy array) to this function. That python array needs to be the joint angles of the robot. Consider getting joint
# angles from the robot using r_limb.get_joint_angles(). This will make it pretty acurate even though the code runs with a theoretical model.
# then you just need to go through the hassle of converting a numpy array to a python array.
# The number you put in after the Fk or the J gets the forward kinematics or the jacobian FROM the base TO the end of whatever link you command.
# I know I said in my little presentation that is goes from one link to the next, but on closer inspection that is not true (sorry I was wrong). It goes from the
# base to the end of the link defined. If you want to get to the end of the 3rd link, you take that number 3, subtract one, and pass that 2 into the brackets.
# this is because python is zero indexed, unlike matlab which is one indexed.
# If you want to find somewhere along the arm, not just the end of the joint, you will have to take different values from this, stubract them from each other
# and interpolate. For example take the position from brk.FK[4]([0, 0, 0, 0, 0, 0, 0]) and subtract from it the position from from brk.FK[2]([0, 0, 0, 0, 0, 0, 0]).
#That will give you the end points of the 3rd arm segment. You can find any point along the line using basic geometry from there.
brk.FK[6]([0, 0, 0, 0, 0, 0, 0])
brk.J[6]([0, 0, 0, 0, 0, 0, 0])

#Final tips
#use np.pi for pi
#use np.dot(mat1,mat2) for matrix multiplication
