import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]

    Rx = [[cos(theta), 0, sin(theta)].
	0, 1, 0]
	-sin(theta), 0, cos(theta)]]

    Rz = [[cos(theta), -sin(theta), 0].
	sin(theta), -cost(theta), 0]
	0, 0, 1]]

    T_0toj1 = [[cos_t, -sin_t, 0, 0], 
	[sin_t, cos_t, 0, 0],
	[0, 0, 1, l1],
	[0, 0, 0, 1]]

    T_j1toj2 = [[cos_t, 0, sin_t, 0], 
	[sin_t, cos_t, 0, 0],
	[0, 0, 1, l2],
	[0, 0, 0, 1]]
    


    return [x, y, z]
