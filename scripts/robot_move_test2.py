#!/usr/bin/env python3

from math import atan, acos, asin, sin, cos, tan
import rospy
# the following line depends upon the
# type of message you are trying to publish
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sympy import symbols, cos, sin, pi, atan2
from sympy.matrices import Matrix
# Define symbolic variables for joint angles
from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
from aruco_msgs.msg import MarkerArray
import time

rospy.init_node('robot_move_test', anonymous=True)

robot_state_publisher = rospy.Publisher(
	"/robot/joint_angles", Float64MultiArray, queue_size=10)

base_link_length = 0.072
l0 = 0.125
l1 = 0.125
l2 = 0.14




def marker_callback(msg):
	for marker in msg.markers:
		if marker.id%10 <= 6:
			print(marker.pose.pose.position)

def move(x,y,theta1,phi,grip):
	my_msg = Float64MultiArray()
	x1 = x 
	y1 = y - base_link_length
	y1 = y1 
	# x2 = 0.17677669529
	# y2 = 0.17677669529
	x2 = x1 - l2*cos(phi)
	y2 = y1 - l2*sin(phi)

	q1 = acos((x2**2 + y2**2 - l0**2 - l1**2)/(2.000*l0*l1))
	q0 = atan(y2/x2) + atan((l1*sin(q1))/(l0+l1*cos(q1)))
	q2 = phi + q1 - q0

	theta1_val = theta1
	theta2_val = q0
	theta3_val = pi/2 - q1
	theta4_val = pi/2 + q2 
	my_msg.data = [0, 0, theta1_val, theta2_val, theta3_val, theta4_val, pi/2,grip]
	robot_state_publisher.publish(my_msg)



if __name__ == '__main__':
	# it is good practice to maintain
	# a 'try'-'except' clause
	# rospy.Subscriber("/aruco_marker_publisher/markers",MarkerArray,marker_callback)
	try:
		
		step1_state = np.array([0.205,0.1,-pi/2,-pi/3,0.05])
		step2_state =  np.array([0.205,0.015,-pi/2,-pi/2,0.05])
		step3_state =  np.array([0.205,0.015,-pi/2,-pi/2,0.0368])
		step4_state =  np.array([0.205,0.1,-pi/2,-pi/3,0.0368])
		step5_state =  np.array([0.205,0.1,-pi,-pi/3,0.0368])
		step6_state =  np.array([0.205,0.1,-pi,-pi/3,0.045])
		steps = 50.00
		state = step1_state
		final_state = step2_state
		diff = (final_state-state)/steps
		rate = rospy.Rate(10)
		i = 0.000
		print(diff)
		while not rospy.is_shutdown() and i < steps:
			state = state + diff
			print(i*diff)
			move(state[0],state[1],state[2],state[3],state[4])
			print(i)
			i +=1 
			rate.sleep()
			
		state = step2_state
		final_state = step3_state
		diff = (final_state-state)/steps
		rate = rospy.Rate(10)
		i = 0.000
		print(diff)
		while not rospy.is_shutdown() and i < steps:
			state = state + diff
			print(i*diff)
			move(state[0],state[1],state[2],state[3],state[4])
			print(i)
			i +=1 
			rate.sleep()
			
		state = step3_state
		final_state = step4_state
		diff = (final_state-state)/steps
		rate = rospy.Rate(10)
		i = 0.000
		print(diff)
		while not rospy.is_shutdown() and i < steps:
			state = state + diff
			print(i*diff)
			move(state[0],state[1],state[2],state[3],state[4])
			print(i)
			i +=1 
			rate.sleep()
			
		state = step4_state
		final_state = step5_state
		diff = (final_state-state)/steps
		rate = rospy.Rate(10)
		i = 0.000
		print(diff)
		while not rospy.is_shutdown() and i < steps:
			state = state + diff
			print(i*diff)
			move(state[0],state[1],state[2],state[3],state[4])
			print(i)
			i +=1 
			rate.sleep()

		state = step5_state
		final_state = step6_state
		diff = (final_state-state)/steps
		rate = rospy.Rate(10)
		i = 0.000
		print(diff)
		while not rospy.is_shutdown() and i < steps:
			state = state + diff
			print(i*diff)
			move(state[0],state[1],state[2],state[3],state[4])
			print(i)
			i +=1 
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
