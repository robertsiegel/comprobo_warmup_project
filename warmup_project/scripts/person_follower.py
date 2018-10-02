#!/usr/bin/env python
from collections import deque
from geometry_msgs.msg import Twist, Vector3
# from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rospy
import time
# import sys, print(sys.version)

class PersonFollowingNode(object):
	def __init__(self):
		rospy.init_node("person_following_node")
		self.person_angle_range = None
		self.person_distance = None
		self.noise_tolerance = .2
		self.distance_tolerance = .2
		self.adjust_tolerance = 20
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def scan_callback(self, msg):
		if self.person_angle_range is None:
			closest = ((0, 0), 0)
			tracking = deque()
			for i, r in enumerate(msg.ranges):
				tracking.append(r)
				tracking_avg = sum(tracking) / len(tracking)
				while abs(tracking[0] - tracking_avg) > tracking_avg * self.noise_tolerance or (tracking_avg < .25 and len(tracking) > 1):
					tracking.popleft()
					tracking_avg = sum(tracking) / len(tracking)
				if tracking_avg > .25 and (tracking_avg < closest[1] or not closest[1]):
					closest = (((i - len(tracking) - 10) % 360, (i - 10) % 360), tracking_avg)
		else:
			search_range = ((self.person_angle_range[0] - 45) % 360, (self.person_angle_range[1] + 45) % 360)
			closest_to_last = ((0, 0), 0)
			tracking = deque()
			debugging = []
			for i in range(*sorted(search_range)):
				debugging.append(msg.ranges[i])
				tracking.append(msg.ranges[i])
				tracking_avg = sum(tracking) / len(tracking)
				while abs(tracking[0] - tracking_avg) > tracking_avg * self.noise_tolerance or (tracking_avg < .25 and len(tracking) > 1):
					tracking.popleft()
					tracking_avg = sum(tracking) / len(tracking)
				if tracking_avg > .25 and (abs(tracking_avg - self.person_distance) < abs(closest_to_last[1] - self.person_distance and abs(tracking_avg - self.person_distance) < self.person_distance * self.distance_tolerance) or (not closest_to_last[1] and abs(tracking_avg - self.person_distance) < self.person_distance / self.distance_tolerance)):
					if abs(tracking_avg - self.person_distance) < self.person_distance * self.distance_tolerance:
						closest_to_last = (((i - len(tracking) - 10) % 360, (i - 10) % 360), tracking_avg)
			closest = closest_to_last
		if closest[1]:
			self.person_angle_range, self.person_distance = closest

	def run(self):
		rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		while not rospy.is_shutdown():
			rospy.loginfo('logging\nperson_angle_range: {}\nperson_distance: {}'.format(self.person_angle_range, self.person_distance))
			if self.person_angle_range and self.person_distance:
				left_fail = self.adjust_tolerance < min(self.person_angle_range)
				right_fail = 360 - self.adjust_tolerance > max(self.person_angle_range)
				rospy.loginfo('left_fail: {} right_fail: {}'.format(left_fail, right_fail))
				if left_fail and right_fail:
					if 360 - max(self.person_angle_range) > min(self.person_angle_range):
						self.vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, .5)))
					else:
						self.vel_pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -.5)))
				else:
					self.vel_pub.publish(Twist(Vector3(.3 * self.person_distance, 0, 0), Vector3(0, 0, 0)))

if __name__ == '__main__':
	node = PersonFollowingNode()
	node.run()