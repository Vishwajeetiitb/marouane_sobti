#!/usr/bin/env python3

from math import atan, acos, asin, sin, cos, tan,atan2
import rospy
# the following line depends upon the
# type of message you are trying to publish
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
# from sympy.matrices import Matrix
# Define symbolic variables for joint angles
# from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
from aruco_msgs.msg import MarkerArray
import time
from gazebo_msgs.msg import LinkStates

rospy.init_node('robot_move_test', anonymous=True)

robot_state_publisher = rospy.Publisher(
	"/robot/joint_angles", Float64MultiArray, queue_size=10)

base_link_length = 0.072
l0 = 0.125
l1 = 0.125
l2 = 0.06

is_cube_detected = False
is_box_detected = False
cube_sample_count = 0
box_sample_count = 0

cube_pose = None
box_pose = None
robot_pose = None




def marker_callback(msg):
	global is_cube_detected,is_box_detected,cube_pose,box_pose,cube_sample_count,box_sample_count
	for marker in msg.markers:
		
		if marker.id%10 <= 6 and not is_cube_detected:
			if cube_pose is not None:
				
				pose = marker.pose.pose.position
				# print(pose)
				cube_pose += np.array([pose.y,pose.x,0.8-pose.z])
				
				if cube_sample_count ==100:
					cube_pose = cube_pose/100
					print("Average cube pose",cube_pose) 
					is_cube_detected = True

				cube_sample_count +=1
			else :
				pose = marker.pose.pose.position
				# print(pose)
				cube_pose = np.array([pose.y,pose.x,0.8-pose.z])
				cube_sample_count +=1 

		if marker.id%10 == 7 and not is_box_detected:
			if box_pose is not None:
				
				pose = marker.pose.pose.position
				# print(pose)
				box_pose += np.array([pose.y,pose.x,0.8-pose.z])
				
				if box_sample_count ==100:
					box_pose = box_pose/100
					print("Average Box pose",box_pose) 
					is_box_detected = True

				box_sample_count +=1
			else :
				pose = marker.pose.pose.position
				# print(pose)
				box_pose = np.array([pose.y,pose.x,0.8-pose.z])
				box_sample_count +=1 


def move(x,y,z,phi,grip):
	my_msg = Float64MultiArray()
	x1 = (x**2 + y**2)**0.5 
	x1 = x1 - (0.04*(x1/abs(x1)))
	y1 = z - base_link_length
	y1 = y1 
	# x2 = 0.17677669529
	# y2 = 0.17677669529
	x2 = x1 - l2*cos(phi)
	y2 = y1 - l2*sin(phi)

	q1 = acos((x2**2 + y2**2 - l0**2 - l1**2)/(2.000*l0*l1))
	q0 = atan2(y2,x2) + atan((l1*sin(q1))/(l0+l1*cos(q1)))
	q2 = phi + q1 - q0

	theta1_val = -atan2(x,y)
	theta2_val = q0
	theta3_val = pi/2 - q1
	theta4_val = pi/2 + q2 
	my_msg.data = [0, 0, theta1_val, theta2_val, theta3_val, theta4_val, pi/2,grip]
	robot_state_publisher.publish(my_msg)

def gazebo_link_states_callback(msg):
	global cube_pose,box_pose,robot_pose,is_box_detected,is_cube_detected
	for i,name in  enumerate(msg.name):
		if 'my_cube' in name:
			Pose = msg.pose[i].position
			cube_pose = [Pose.x,Pose.y,Pose.z]
			is_cube_detected = True

		if 'my_box' in name:
			Pose = msg.pose[i].position
			box_pose = [Pose.x,Pose.y,Pose.z]
			is_box_detected = True

		if 'robot_base_link' in name:
			Pose = msg.pose[i].position
			robot_pose = [Pose.x,Pose.y,Pose.z]


if __name__ == '__main__':
	# it is good practice to maintain
	# a 'try'-'except' clause
    
	rospy.wait_for_message("/gazebo/link_states",LinkStates)
	print("Waiting for 5 seconds")
	time.sleep(5)
	# rospy.Subscriber("/aruco_marker_publisher/markers",MarkerArray,marker_callback)
	rospy.Subscriber("/gazebo/link_states",LinkStates,gazebo_link_states_callback)
	try:
		while not rospy.is_shutdown():
			if is_cube_detected and is_box_detected:
				
				state0 = np.array([0.0,0.125,0.06,0,0.03])
				state1 = np.array([cube_pose[0],cube_pose[1],cube_pose[2]+0.06,-pi/3,0.06])
				state2 = np.array([cube_pose[0],cube_pose[1],cube_pose[2],-pi/2,0.06])
				state3 = np.array([cube_pose[0],cube_pose[1],cube_pose[2],-pi/2,0.0368])
				state4 = np.array([cube_pose[0],cube_pose[1],cube_pose[2]+0.06,-pi/2,0.0368])
				state5 = np.array([0.01,0.01,0.3,pi/2,0.0368])
				state6 = np.array([box_pose[0],box_pose[1],box_pose[2]+0.2,-pi/3,0.0368])
				print(box_pose)
				state7 = np.array([box_pose[0],box_pose[1],box_pose[2]+0.2,-pi/3,0.045])
				step_size = 100
				rate = rospy.Rate(30)
				states = [state1,state2,state3,state4,state5,state6,state7]
				current_state = state0
				for i in range(100):
					move(current_state[0],current_state[1],current_state[2],current_state[3],current_state[4])
					rate.sleep()
				time.sleep(5)
				for m,next_state in enumerate(states):
					i = 0
					step = (next_state-current_state)/step_size
					print(step*step_size)
					while not rospy.is_shutdown() and i <step_size:
						move(current_state[0],current_state[1],current_state[2],current_state[3],current_state[4])
						i +=1
						current_state =current_state + step
						rate.sleep()
						current_state_state = next_state
				exit()
            
			
		# state = step2_state
		# final_state = step3_state
		# diff = (final_state-state)/steps
		# rate = rospy.Rate(10)
		# i = 0.000
		# print(diff)
		# while not rospy.is_shutdown() and i < steps:
		# 	state = state + diff
		# 	print(i*diff)
		# 	move(state[0],state[1],state[2],state[3],state[4])
		# 	print(i)
		# 	i +=1 
		# 	rate.sleep()
			
		# state = step3_state
		# final_state = step4_state
		# diff = (final_state-state)/steps
		# rate = rospy.Rate(10)
		# i = 0.000
		# print(diff)
		# while not rospy.is_shutdown() and i < steps:
		# 	state = state + diff
		# 	print(i*diff)
		# 	move(state[0],state[1],state[2],state[3],state[4])
		# 	print(i)
		# 	i +=1 
		# 	rate.sleep()
			
		# state = step4_state
		# final_state = step5_state
		# diff = (final_state-state)/steps
		# rate = rospy.Rate(10)
		# i = 0.000
		# print(diff)
		# while not rospy.is_shutdown() and i < steps:
		# 	state = state + diff
		# 	print(i*diff)
		# 	move(state[0],state[1],state[2],state[3],state[4])
		# 	print(i)
		# 	i +=1 
		# 	rate.sleep()

		# state = step5_state
		# final_state = step6_state
		# diff = (final_state-state)/steps
		# rate = rospy.Rate(10)
		# i = 0.000
		# print(diff)
		# while not rospy.is_shutdown() and i < steps:
		# 	state = state + diff
		# 	print(i*diff)
		# 	move(state[0],state[1],state[2],state[3],state[4])
		# 	print(i)
		# 	i +=1 
		# 	rate.sleep()
	except rospy.ROSInterruptException:
		pass
