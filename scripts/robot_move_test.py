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


rospy.init_node('robot_move_test', anonymous=True)

robot_state_publisher = rospy.Publisher(
	"/robot/joint_angles", Float64MultiArray, queue_size=10)

base_link_length = 0.072
l0 = 0.125
l1 = 0.125
l2 = 0.14
phi = -pi/2

x1 = 0.2
y1 = 0.03 - base_link_length
y1 = y1 
# x2 = 0.17677669529
# y2 = 0.17677669529
x2 = x1 - l2*cos(phi)
y2 = y1 - l2*sin(phi)


q1 = acos((x2**2 + y2**2 - l0**2 - l1**2)/(2.000*l0*l1))
print(x2/y2)
q0 = atan(y2/x2) + atan((l1*sin(q1))/(l0+l1*cos(q1)))
q2 = phi + q1 - q0

theta1_val = -pi/2
theta2_val = q0
theta3_val = pi/2 - q1
theta4_val = pi/2 + q2 

def marker_callback(msg):
	for marker in msg.markers:
		if marker.id%10 <= 6:
			print(marker.pose.pose.position)

# def move(x,y,z):
	


if __name__ == '__main__':
	# it is good practice to maintain
	# a 'try'-'except' clause
	rospy.Subscriber("/aruco_marker_publisher/markers",MarkerArray,marker_callback)
	try:
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			data = pi/4

			# you could simultaneously display the data
			# on the terminal and to the log file
			# rospy.loginfo(data)

			# publish the data to the topic using publish()
			# x_pose_pub.publish(0)
			# y_pose_pub.publish(0)

			# joint1_pub.publish(theta1)
			# joint2_pub.publish(theta2)
			# joint3_pub.publish(theta3)
			# joint4_pub.publish(theta4)
			my_msg = Float64MultiArray()
			# print("inverse is successful: {0}".format(robot.is_reachable_inverse))
			# print(theta1_val,theta2_val,theta3_val,theta4_val)
			# print(theta3_val)
			# my_msg.data = [0, 0, theta1_val, theta2_val, theta3_val, 0, pi/2,0.005]
			my_msg.data = [0.00, 0, -pi, 1.57, 0, 0.33, pi/2,0.055]
			# my_msg.data = [-3.6477647659688477e-05, 0.0006032144530847963, 0.00030949820340619993, pi/2, 0.0200257221690061, 1.5395564580119547, 0.019991744738777163, 0.0002473360219621412, -4.9956027350361865e-06]
			robot_state_publisher.publish(my_msg)

			# keep a buffer based on the rate defined earlier
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
