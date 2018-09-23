#!/usr/bin/env python
from collections import deque
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
		vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		ranges = msg.ranges
		# # first and last format tuple(index, val)
		# first = None
		# last = None
		# front_sum = 0
		# front_count = 0
		# running_vals = deque([0 for _ in range(10)])
		# running_avg = 0
		# delayed_running_vals = deque([0 for _ in range(10)])
		# delayed_running_avg = 0
		# lag_amt = 10
		# debugging = deque()
		# for i in range(90, -91, -1):
		# 	index = i if i > 0 else 360 + i
		# 	debugging.append(ranges[index])
		# 	''' average shit here'''
		# 	if abs(ranges[index] - running_avg) < running_avg / 2 or (ranges[index] == 0 and running_avg < .15):
		# 	# 	pass
		# 	# else:
		# 		lagged_index = i + lag_amt if i > -lag_amt else 360 + lag_amt + i
		# 		if first is None and ranges[index] > 0:
		# 			first = (index, running_avg)
		# 		elif first and last is None and ranges[index] == 0:
		# 			# last = (index - lag_amt, ranges[index - lag_amt])
		# 			last = (lagged_index, delayed_running_avg)
		# 		if i in range(-22, 22):
		# 			front_sum += ranges[index]
		# 			front_count += 1
		# 	# running_avg -= running_vals.popleft() / 10
		# 	# running_vals.append(ranges[index])
		# 	# running_avg += ranges[index] / 10
		# 	moved_val = running_vals.popleft()
		# 	running_avg -= moved_val / 10
		# 	running_vals.append(ranges[index])
		# 	running_avg += ranges[index] / 10
		# 	delayed_running_avg -= delayed_running_vals.popleft() / 10
		# 	delayed_running_vals.append(moved_val)
		# 	delayed_running_avg += moved_val / 10
		# front_avg = front_sum / front_count if front_count else 0
		# rospy.loginfo('first: {}\nlast: {}\navg: {}'.format(first, last, front_avg))
		# # rospy.loginfo(debugging)
		# # rospy.loginfo(len(ranges))
		# if first and last and front_avg:
		# 	# turn left case
		# 	if first[1] > last[1]:
		# 		turn = .5 / front_avg
		# 	# turn right case
		# 	else:
		# 		turn = -.5 / front_avg
		# 	time.sleep(.25)

		# 	vel_msg = Twist(Vector3(.25, 0, 0), Vector3(0, 0, turn))
		# 	vel_pub.publish(vel_msg)
		# else:
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
		# else:
			# vel_pub.publish(Twist(Vector3(.25, 0, 0), Vector3(0, 0, 0)))
		# rospy.loginfo(msg.ranges)

	def listener(self):
		scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		rospy.spin()


if __name__ == '__main__':
	node = FollowWallNode()
	node.listener()