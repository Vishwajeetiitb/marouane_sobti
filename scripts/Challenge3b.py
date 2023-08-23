#!/usr/bin/python3

from math import atan, acos, asin, sin, cos, tan,atan2
import rospy
# the following line depends upon the
# type of message you are trying to publish
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sympy.matrices import Matrix
# Define symbolic variables for joint angles
from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi
from aruco_msgs.msg import MarkerArray
import time
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates

rospy.init_node('robot_move_test', anonymous=True)

robot_state_publisher = rospy.Publisher(
	"/robot/joint_angles", Float64MultiArray, queue_size=10)

base_link_length = 0.072
l0 = 0.125
l1 = 0.125
l2 = 0.1

is_cube_detected = False
is_box_detected = False
is_box_obstacle_detected = False
cube_sample_count = 0
box_sample_count = 0

cube_pose_list = None
cube_seqeunce = None
box_pose = None
robot_pose = None
arm_state = [0.0001,0.0001,0.4219,pi/2,False]



def marker_callback(msg):
	global is_box_obstacle_detected,robot_pose
	for marker in msg.markers:
		if marker.id%10 == 7:
			pose = marker.pose.pose.position
			box_pose = [pose.y,pose.x,0.8-pose.z]
			distance = ((box_pose[0])**2 + (box_pose[1])**2)**0.50
			if distance <=0.3:
				is_box_obstacle_detected = True
				print("Obstacle!",box_pose)
			


def gazebo_model_states_callback(msg):
	global cube_pose,box_pose,robot_pose,is_box_detected,is_cube_detected
	for i,name in  enumerate(msg.name):
		if 'cube' in name:
			Pose = msg.pose[i].position
			cube_pose = [Pose.x,Pose.y,Pose.z]
			is_cube_detected = True

		if 'box' in name:
			Pose = msg.pose[i].position
			box_pose = [Pose.x,Pose.y,Pose.z]
			is_box_detected = True

		if name == 'robot':
			Pose = msg.pose[i].position
			robot_pose = [Pose.x,Pose.y,Pose.z]

def gazebo_link_states_callback(msg):
	global box_pose,robot_pose,is_box_detected,is_cube_detected,cube_pose_list,cube_seqeunce
	cube_pose_list = []
	cube_seqeunce = []
	
	for i,name in  enumerate(msg.name):
		if 'my_cube' in name:
			id = int(name.split('my_cube')[1])
			if id not in cube_seqeunce:
				cube_seqeunce.append(id)
				Pose = msg.pose[i].position
				cube_pose_list.append([Pose.x,Pose.y,Pose.z])			

		if 'my_box' in name:
			Pose = msg.pose[i].position
			box_pose = [Pose.x,Pose.y,Pose.z]
			is_box_detected = True

		if 'robot_base_link' in name:
			Pose = msg.pose[i].position
			robot_pose = [Pose.x,Pose.y,Pose.z]
		is_cube_detected = True

def move(x,y,z,phi,grip,base_x,base_y):
	global arm_state
	my_msg = Float64MultiArray()
	x_offset = 0.00
	y_offset = 0.0
	x = x + x_offset
	y = y + y_offset
	x1 = (x**2 + y**2)**0.500
	# x1 = x1 - (0.04*(x1/abs(x1)))
	y1 = z - base_link_length
	y1 = y1 
	x2 = x1 - l2*cos(phi)
	y2 = y1 - l2*sin(phi)

	q1 = acos((x2**2 + y2**2 - l0**2 - l1**2)/(2.000*l0*l1))
	q0 = atan2(y2,x2) + atan((l1*sin(q1))/(l0+l1*cos(q1)))
	q2 = phi + q1 - q0

	theta1_val = -atan2(x,y)
	theta2_val = q0
	theta3_val = pi/2 - q1
	theta4_val = pi/2 + q2 
	grip_val = 0.0368 if grip else 0.06
	
	my_msg.data = [base_x, base_y, theta1_val, theta2_val, theta3_val, theta4_val, pi/2,]
	arm_state = [x,y,z,phi,grip]
	robot_state_publisher.publish(my_msg)

# def go_towards_cube(robot_pose,cube_pose):
	
def arm_at_rest(gripped):
	global robot_pose
	move(0.0001,0.0001,0.4219,pi/2,gripped,robot_pose[0],robot_pose[1])

def go_to(target_x,target_y,gripped):
	global arm_state
	current_x = robot_pose[0]
	current_y = robot_pose[0]
	m = (target_y-current_y)/(target_x-current_x)
	b = (current_x*target_y - target_x*current_y)/(current_x-target_x)
	x = np.linspace(current_x,target_x,100)
	y = m*x + b
	rate = rospy.Rate(15)
	for x_coordinate,y_coordinate in zip(x,y):
		move(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],x_coordinate,y_coordinate)
		move(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],x_coordinate,y_coordinate)
		if not is_box_obstacle_detected:
			move(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],x_coordinate,y_coordinate)
			move(arm_state[0],arm_state[1],arm_state[2],arm_state[3],arm_state[4],x_coordinate,y_coordinate)
		else
		# else: print("Obstacle!")

def grip_cube(x,y,z):
	
	move(x,y,z,-pi/2,False,robot_pose[0],robot_pose[1])
	

if __name__ == '__main__':
	# it is good practice to maintain
	# a 'try'-'except' clause
	
	rospy.wait_for_message("/gazebo/link_states",LinkStates)
	# print("Waiting for 5 seconds")
	
	rospy.Subscriber("/aruco_marker_publisher/markers",MarkerArray,marker_callback)
	# rospy.Subscriber("/gazebo/model_states",ModelStates,gazebo_model_states_callback)
	rospy.Subscriber("/gazebo/link_states",LinkStates,gazebo_link_states_callback)
	time.sleep(2)
	rate = rospy.Rate(1)
	go_to(-1,-1,1)
	while not rospy.is_shutdown():
		# move(0.0001,0.0001,0.4219,pi/2,0,0,0)
	# 	go_to(0,0,1)
	# 	# grip_cube(0.2,0.01,0.06)
	# 	# arm_at_rest(1)
    
		rate.sleep()
	
	# rospy.sleep(100)
	try:
		while not rospy.is_shutdown():
			if is_box_detected:
				while not is_cube_detected : print("waiting for cube detection")
				time.sleep(1)
				
				state0 = np.array([0.0,0.125,0.06,0,0.03,0,0])
				cube_seqeunce_sorted = sorted(cube_seqeunce)
				indices = [cube_seqeunce.index(i) for i in cube_seqeunce_sorted]
				print(cube_seqeunce,indices)
				cube_pose_list = [cube_pose_list[i] for i in indices ]
				cube_seqeunce = cube_seqeunce_sorted
				
				for id,cube_pose in zip(cube_seqeunce,cube_pose_list):
					if is_cube_detected:
						print(id,cube_pose)
						state0 = np.array([0.0,0.125,0.06,0,0.03,cube_pose[0]-0.2*(cube_pose[0]/abs(cube_pose[0])),cube_pose[1]])
						state1 = np.array([0.2*(cube_pose[0]/abs(cube_pose[0])),0.01,cube_pose[2]+0.06,-pi/3,False,cube_pose[0]-0.2*(cube_pose[0]/abs(cube_pose[0])),cube_pose[1]])
						state2 = np.array([0.2*(cube_pose[0]/abs(cube_pose[0])),0.01,cube_pose[2],-pi/2,False,cube_pose[0]-0.2*(cube_pose[0]/abs(cube_pose[0])),cube_pose[1]])
						state3 = np.array([0.2*(cube_pose[0]/abs(cube_pose[0])),0.01,cube_pose[2],-pi/2,False,cube_pose[0]-0.2*(cube_pose[0]/abs(cube_pose[0])),cube_pose[1]])
						state4 = np.array([0.2*(cube_pose[0]/abs(cube_pose[0])),0.01,cube_pose[2]+0.06,-pi/2,True,cube_pose[0]-0.2*(cube_pose[0]/abs(cube_pose[0])),cube_pose[1]])
						state5 = np.array([0.01,0.01,0.3,pi/2,True,cube_pose[0]-0.2*(cube_pose[0]/abs(cube_pose[0])),cube_pose[1]])
						state6 = np.array([0.01,0.01,0.3,pi/2,True,box_pose[0]-(0.2*(box_pose[0]/abs(box_pose[0]))),box_pose[1]])
						state7 = np.array([0.2*(box_pose[0]/abs(box_pose[0])),0.01,0.2,-pi/3,True,box_pose[0]-(0.2*(box_pose[0]/abs(box_pose[0]))),box_pose[1]])
						state8 = np.array([0.2*(box_pose[0]/abs(box_pose[0])),0.01,0.2,-pi/3,False,box_pose[0]-(0.2*(box_pose[0]/abs(box_pose[0]))),box_pose[1]])
						if id == 1:
							current_state = state0
							step_sizes = [100,100,100,100,100,500,100,5]
							states = [state1,state2,state3,state4,state5,state6,state7,state8]
						else :
							step_sizes = [600,100,100,100,100,100,500,100,5]
							states = [state0,state1,state2,state3,state4,state5,state6,state7,state8]
							
							
							
						rate = rospy.Rate(25)
						
						for i in range(50):
							move(current_state[0],current_state[1],current_state[2],current_state[3],current_state[4],current_state[5],current_state[6])
							rate.sleep()
                            
                        
						time.sleep(2)
						for step_size,next_state in zip(step_sizes,states):
							i = 0
							step = (next_state-current_state)/step_size
							while not rospy.is_shutdown() and i <step_size:
								move(current_state[0],current_state[1],current_state[2],current_state[3],current_state[4],current_state[5],current_state[6])
								i +=1
								current_state =current_state + step
								rate.sleep()
								current_state_state = next_state
						
					
				exit()
			

	except rospy.ROSInterruptException:
		pass
