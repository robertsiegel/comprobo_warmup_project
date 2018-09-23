#!/usr/bin/env python
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import time

class FollowWallNode(object):
	""" This node drives the neato in a square """
	def __init__(self):
		rospy.init_node("follow_wall_node")
		# self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.start_angle = -1.57
		self.end_angle = 1.57

	def callback(self, msg):
		# while not rospy.is_shutdown():
			# scan = LaserScan()
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		ranges = msg.ranges
		# first and last format tuple(index, val)
		first = None
		last = None
		front_sum = 0
		for i in range(90, -91, -1):
			index = i if i > 0 else 360 + i
			if first is None and ranges[index] > 0:
				first = (index, ranges[index])
			elif first and last is None and ranges[index] == 0:
				last = (index - 1, ranges[index - 1])
			if i in range(-22, 22):
				front_sum += ranges[index]
		front_avg = front_sum / 45
		rospy.loginfo('first: {}\nlast: {}\navg: {}'.format(first, last, front_avg))
		if first and last and front_avg:
			# turn left case
			if first[1] > last[1]:
				turn = .05 / front_avg
			# turn right case
			else:
				turn = -.05 / front_avg

			vel_msg = Twist(Vector3(.25, 0, 0), Vector3(0, 0, turn))
			# vel_msg.linear.x = .25
			# vel_msg.linear.y = 0 
			# vel_msg.linear.z = 0
			# vel_msg.angular.x = 0
			# vel_msg.angular.y = 0
			# vel_msg.angular.z = turn
			# self.vel_pub.publish(vel_msg)
			vel_pub.publish(vel_msg)
		else:
			vel_pub.publish(Twist(Vector3(.25, 0, 0), Vector3(0, 0, 0)))
		# rospy.loginfo(msg.ranges)

	def listener(self):
		scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()


if __name__ == '__main__':
	node = FollowWallNode()
	node.listener()