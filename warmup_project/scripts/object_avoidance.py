#!/usr/bin/env python
from collections import deque
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import time

class FollowPersonNode(object):
	""" This node follows a person """
	def __init__(self):
		rospy.init_node("follow_person_node")
		# self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.start_angle = -1.57
		self.end_angle = 1.57

	def callback(self, msg):
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		ranges = msg.ranges
		# 	vel_pub.publish(Twist(Vector3(.25, 0, 0), Vector3(0, 0, 0)))
		# # rospy.loginfo(msg.ranges)
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

		left_avg = sum(left) / len(left)
		right_avg = sum(right) / len(right)
		middle_avg = sum(middle) / len(middle)
		turn = 0
		if len(left) < 30:
			if len(right) < 30:
				pass
			else:
				turn = .5 / middle_avg
		elif len(right) < 30:
			turn = -.5 / middle_avg
		elif left_avg > right_avg:
			turn = .5 / middle_avg
		elif right_avg > left_avg:
			turn = -.5 / middle_avg
		else:
			rospy.loginfo("wtf... \nleft_avg: {}\nright_avg{}".format(left_avg, right_avg))
		
		vel_msg = Twist(Vector3(.25, 0, 0), Vector3(0, 0, turn))
		vel_pub.publish(vel_msg)
		time.sleep(.25)
		# rospy.loginfo(msg.ranges)

	def listener(self):
		scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()


if __name__ == '__main__':
	node = FollowWallNode()
	node.listener()