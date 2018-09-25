#!/usr/bin/env python
from collections import deque
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import rospy
import time
import numpy as np
import cv2 

class FollowWallNode(object):
	""" This node drives the neato to follow a wall """
	def __init__(self):
		rospy.init_node("follow_wall_node")
		# self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.start_angle = -1.57
		self.end_angle = 1.57


	def find_wall(self, ranges):
		vis_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
		
		img_dim = 100

		img = np.zeros([img_dim,img_dim,3],dtype=np.uint8)
		img.fill(255) # or img[:] = 255

		for i in range(90, -91, -1):
			index = i if i > 0 else 360 + i
		
			if ranges[index]:
				cartPoint = self.polar_to_cart((index, ranges[index]))
				img[(img_dim/2)+(cartPoint[0]*10), (img_dim/2)+(cartPoint[1]*10)] = [0,0,0]
		
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
		edges = cv2.Canny(gray,50,150,apertureSize = 3) 

		lines = cv2.HoughLinesP(edges,1,np.pi/180, 10) 
		rospy.loginfo("lines: {}".format(lines))

		if lines.any():	
			for line in lines:
				x1,y1,x2,y2 = line[0]
				cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
		
		cv2.imwrite('03.png', img)

		# for line in lines:
		# 	pt1 = [line[0], line[1]];
		# 	pt2 = [line[2], line[3]];
		# 	outImage.line(pt1, pt2, RED); 

		# if cartPoint:
		# 	first_wall_point = Point()
		# 	first_wall_point.x = cartesianWall[0]
		# 	first_wall_point.y = cartesianWall[1]
		# 	first_wall_point.z = 0
		# 	last_wall_point = Point()
		# 	last_wall_point.x = cartesianWall[2]
		# 	last_wall_point.y = cartesianWall[3]
		# 	last_wall_point.z = 0


		# 	vis_msg = Marker()
		# 	vis_msg.type = 4
		# 	vis_msg.header.frame_id = "odom"
		# 	vis_msg.points = [first_wall_point, last_wall_point]
			# vis_msg.pose.position.x = cartesianWall[0]
			# vis_msg.pose.position.y = cartesianWall[1]
			# vis_msg.pose.position.z = 0
			# vis_msg.pose.orientation.x = 0.0
			# vis_msg.pose.orientation.y = 0.0
			# vis_msg.pose.orientation.z = 0.0
			# vis_msg.pose.orientation.w = 1.0
			# vis_msg.scale.x = 1;
			# vis_msg.scale.y = 0.1
			# vis_msg.scale.z = 0.1
			# vis_msg.color.a = 1.0
			# vis_msg.color.r = 0.0
			# vis_msg.color.g = 1.0
			# vis_msg.color.b = 0.0
			# vis_pub.publish(vis_msg)



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
		# rospy.loginfo(ranges)
		# for i in range(90, -90 -1):
		# 	rospy.loginfo(i)
		# 	index = i if i > 0 else 360 + i
		# 	rospy.loginfo(index)
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
		vel_pub.publish(vel_msg)
		self.find_wall(ranges)
		time.sleep(.25)
		# else:
			# vel_pub.publish(Twist(Vector3(.25, 0, 0), Vector3(0, 0, 0)))
		# rospy.loginfo(msg.ranges)

	def listener(self):
		scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()


if __name__ == '__main__':
	node = FollowWallNode()
	node.listener()