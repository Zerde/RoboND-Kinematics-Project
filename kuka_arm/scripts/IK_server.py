#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


# Define Modified DH Transformation matrix
def transformation_matrix(i, a, d, q):
    TF_Matrix =  Matrix([[        cos(q),       -sin(q),       0,         a],
                         [ sin(q)*cos(i), cos(q)*cos(i), -sin(i), -sin(i)*d],
                         [ sin(q)*sin(i), cos(q)*sin(i),  cos(i),  cos(i)*d],
                         [             0,             0,       0,         1]])
    return TF_Matrix


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angle
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
        i0, i1, i2, i3, i4, i5, i6 = symbols('i0:7') # twist angle/ alpha

        # DH Parameters
        s = {i0:     0, a0:      0, d1:  0.75,
             i1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
             i2:     0, a2:   1.25, d3:     0,
             i3: -pi/2, a3: -0.054, d4:  1.50,
             i4:  pi/2, a4:      0, d5:     0,
             i5: -pi/2, a5:      0, d6:     0,
             i6:     0, a6:      0, d7: 0.303, q7: 0}
       

	# Create individual transformation matrices
        T0_1  = transformation_matrix(i0, a0, d1, q1).subs(s)
        T1_2  = transformation_matrix(i1, a1, d2, q2).subs(s)
        T2_3  = transformation_matrix(i2, a2, d3, q3).subs(s)
        #T3_4  = transformation_matrix(i3, a3, d4, q4).subs(s)
        #T4_5  = transformation_matrix(i4, a4, d5, q5).subs(s)
        #T5_6  = transformation_matrix(i5, a5, d6, q6).subs(s)
        #T6_EE = transformation_matrix(i6, a6, d7, q7).subs(s)

        #T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE 
		# Extract rotation matrices from the transformation matrices

        r, p, yw = symbols('r p yw') 

        R_x = Matrix([[      1,         0,        0],
                      [      0,     cos(r), -sin(r)],
                      [      0,     sin(r),  cos(r)]])
        R_y = Matrix([[ cos(p),          0,  sin(p)],
                      [      0,          1,       0],
                      [-sin(p),          0,  cos(p)]])
        R_z = Matrix([[ cos(yw),  -sin(yw),       0],
                      [ sin(yw),   cos(yw),       0],
                      [       0,         0,       1]])
        R_EE = R_z * R_y * R_x
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        R_err = R_z.subs(yw, np.pi) * R_y.subs(p,-np.pi/2)
        R_EE = R_EE * R_err

        #rotational matrix first three joints
        R0to3 = T0_1[0:3, 0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
               
	
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	        print("X:" + str(px) + "; Y:" + str(py) + "; Z:" + str(pz))
            
            R_EE = R_EE.subs({'r': roll, 'p': pitch, 'yw' : yaw})
	        print("Orientation x:" + str(req.poses[x].orientation.x) + "; y:" + str(req.poses[x].orientation.y) + "; z:" + str(req.poses[x].orientation.z) + "; w:" + str(req.poses[x].orientation.w))
            EE = Matrix([[px],
                         [py],
                         [pz]])
            # wrist center coordinates
            WC = EE - (0.303) * R_EE[:,2]
           
            # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            # calculating triangle sides and angles for theta2 and theta3
            a = sqrt(0.054*0.054 + 1.5*1.5)
            b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35),2) + pow((0.75 - WC[2]),2))
            c = 1.25

            half_per = (a + b + c)/2
            area = sqrt(half_per * (half_per - a) * (half_per - b) * (half_per - c))

            alpha = atan2((4*area), (b*b + c*c - a*a))
            beta  = atan2((4*area),(a*a + c*c - b*b))

            theta2 = np.pi/2 - (alpha - atan2((0.75 - WC[2]), (sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35)))
            theta3 = atan2(1.5,0.054) - beta

            R0_3 = R0to3.evalf(subs={q1:theta1, q2: theta2, q3:theta3})

            R3_6 = R0_3.transpose() * R_EE

            #Euler Angles from Rotation Matix
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
    
            if sin(theta5) > 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2]) 
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2]) 
                theta6 = atan2( R3_6[1,1], -R3_6[1,0]) 
            
            print(str(theta1) + "|" + str(theta2) + "|" + str(theta3) + "|" + str(theta4) + "|" + str(theta5) + "|" + str(theta6))
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
