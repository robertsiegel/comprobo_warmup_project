#!/usr/bin/env python
from collections import deque
from geometry_msgs.msg import Twist, Vector3, Point, Pose
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import rospy
import time
import numpy as np
import cv2 
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import math

class FollowWallNode(object):
	""" This node drives the neato to follow a wall """
	def __init__(self):
		rospy.init_node("follow_wall_node")
		# self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def find_wall(self, ranges):
		vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

		img_size = 100.0
		pt_multpilier = 10.0

		img = np.zeros([img_size,img_size,3],dtype=np.uint8)
		img.fill(255) # or img[:] = 255

		for i in range(90, -91, -1):
			index = i if i > 0 else 360 + i
		
			if ranges[index]:
				cartPoint = self.polar_to_cart((index, ranges[index]))
				img[(img_size/2)+(cartPoint[0]*pt_multpilier), (img_size/2)+(cartPoint[1]*pt_multpilier)] = [0,0,0]
		
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
		edges = cv2.Canny(gray,50,150,apertureSize = 3) 

		lines = cv2.HoughLinesP(edges,1,np.pi/180, 10) 

		neato_xy = self.convert_pose_to_xy_and_theta(Pose())

		longest_length = 0
		if lines.any():
			for line in lines:
				x1,y1,x2,y2 = line[0]
				x1 = (x1-(img_size/2))/pt_multpilier-neato_xy[0]
				y1 = (y1-(img_size/2))/pt_multpilier-neato_xy[1]
				x2 = (x2-(img_size/2))/pt_multpilier-neato_xy[0]
				y2 = (y2-(img_size/2))/pt_multpilier-neato_xy[1]

				if abs(math.sqrt((x2-x1)**2+(y2-y1)**2)) > longest_length:
					first_wall_point = Point()
					first_wall_point.x = x1
					first_wall_point.y = y1
					first_wall_point.z = 0
					last_wall_point = Point()
					last_wall_point.x = x2
					last_wall_point.y = y2
					last_wall_point.z = 0
					rospy.loginfo("Line points: {},\n{}".format(first_wall_point, last_wall_point))


	def polar_to_cart(self, point):
		angle = np.deg2rad(point[0])
		if point: 
			x = point[1] * np.cos(angle)
	 		y = point[1] * np.sin(angle)
	 		return (x,y)
	 	else:
	 		return None

	def callback(self, msg):
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
		ranges = msg.ranges


		left = []
		middle = []
		right = []

		for index in range(0, 361):
			if index in range(30, 91) and ranges[index]:
				left.append(ranges[index])
			elif index in range(0, 30) or index in range(330, 361) and ranges[index]:
				middle.append(ranges[index])
			elif index in range(270, 330) and ranges[index]:
				right.append(ranges[index])

		left_avg = sum(left) / len(left) if len(left) else 0
		right_avg = sum(right) / len(right) if len(right) else 0
		middle_avg = sum(middle) / len(middle) if len(middle) and sum(middle) else 10

		turn = 0
		if len(left) < 30:
			if len(right) < 30:
				pass
			elif right_avg < .75:
				pass
			else:
				turn = .5 / middle_avg
		elif len(right) < 30:
			if left_avg < .75:
				pass
			else:
				turn = -.5 / middle_avg
		elif left_avg > right_avg:
			turn = .5 / middle_avg
		elif right_avg > left_avg:
			turn = -.5 / middle_avg
		else:
			rospy.loginfo("wtf... \nleft_avg: {}\nright_avg{}".format(left_avg, right_avg))
		
		vel_msg = Twist(Vector3(.25, 0, 0), Vector3(0, 0, turn))
		# vel_pub.publish(vel_msg)
		self.find_wall(ranges)
		time.sleep(.25)
		# else:
			# vel_pub.publish(Twist(Vector3(.25, 0, 0), Vector3(0, 0, 0)))
		# rospy.loginfo(msg.ranges)

	def listener(self):
		scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()


	def convert_pose_to_xy_and_theta(self, pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x,
							 pose.orientation.y,
							 pose.orientation.z,
							 pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])

if __name__ == '__main__':
	node = FollowWallNode()
	node.listener()