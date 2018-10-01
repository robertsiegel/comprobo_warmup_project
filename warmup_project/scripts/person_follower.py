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
		# self.dir = None
		# self.dir_offset = None
		self.person_angle_range = None
		self.person_distance = None
		self.noise_tolerance = .2
		self.distance_tolerance = .2

	# def dir_callback(self, msg):
	# 	pass

	def scan_callback(self, msg):
		rospy
		if self.person_angle_range is None:
			# rospy.loginfo(msg.ranges)
			# find the closest immediate object
			# closest format: Tuple(Tuple(begining angle, end angle), distance)
			closest = ((0, 0), 0)
			tracking = deque()
			for i, r in enumerate(msg.ranges):
				tracking.append(r)
				tracking_avg = sum(tracking) / len(tracking)
				# rospy.loginfo('{}\n{}'.format(tracking, tracking_avg))
				# can probably only count as tracked object if there's enough points for it
				'''
				update this shit for that
				'''
				while abs(tracking[0] - tracking_avg) > tracking_avg * self.noise_tolerance or (tracking_avg < .25 and len(tracking) > 1):
					tracking.popleft()
					tracking_avg = sum(tracking) / len(tracking)
				if tracking_avg > .25 and (tracking_avg < closest[1] or not closest[1]):
					closest = (((i - len(tracking)) % 360, i), tracking_avg)
			# if closest[1]:
			# 	self.person_angle_range, self.person_distance = closest
		else:
			# find the same person that was previously being tracked
			search_range = ((self.person_angle_range[0] - 45) % 360, (self.person_angle_range[1] + 45) % 360)
			closest_to_last = ((0, 0), 0)
			tracking = deque()
			debugging = []
			for i in range(*sorted(search_range)):
				debugging.append(msg.ranges[i])
				tracking.append(msg.ranges[i])
				tracking_avg = sum(tracking) / len(tracking)
				# rospy.loginfo('{}\n{}'.format(tracking, tracking_avg))
				while abs(tracking[0] - tracking_avg) > tracking_avg * self.noise_tolerance or (tracking_avg < .25 and len(tracking) > 1):
					tracking.popleft()
					tracking_avg = sum(tracking) / len(tracking)
				# rospy.loginfo('comparing distances: old: {} new: {}'.format(abs(closest_to_last[1] - self.person_distance), abs(tracking_avg - self.person_distance)))
				# if not closest_to_last[1]:
				# 	rospy.loginfo('checking distance and tolerance: ')
				# update closest if current tracked is closer to the previous person location than the closest_to_last, or if closest_to_last hasn't been updated and current tracked is within distance tolerance
				if tracking_avg > .25 and (abs(tracking_avg - self.person_distance) < abs(closest_to_last[1] - self.person_distance and abs(tracking_avg - self.person_distance) < self.person_distance * self.distance_tolerance) or (not closest_to_last[1] and abs(tracking_avg - self.person_distance) < self.person_distance / self.distance_tolerance)):
					# rospy.loginfo('closest_to_last: {}'.format(closest_to_last))
					# rospy.loginfo('comparing distances: old: {} new: {}'.format(abs(closest_to_last[1] - self.person_distance), abs(tracking_avg - self.person_distance)))
					if abs(tracking_avg - self.person_distance) < self.person_distance * self.distance_tolerance:
						closest_to_last = (((i - len(tracking)) % 360, i), tracking_avg)
			closest = closest_to_last
			# rospy.loginfo('search range: {}\nvals: {}'.format(search_range, debugging))
		# self.person_angle_range = closest[0]
		# self.person_distance = closest[1]
		# rospy.loginfo(msg.ranges)
		if closest[1]:
			self.person_angle_range, self.person_distance = closest
		# time.sleep(1)


	def run(self):
		rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		# rospy.Subscriber('/scan', Odometry, self.dir_callback)
		while not rospy.is_shutdown():
			# TODO: implement acting on scan callbacks
			# pass
			rospy.loginfo('logging\nperson_angle_range: {}\nperson_distance: {}'.format(self.person_angle_range, self.person_distance))
			# time.sleep(1)



if __name__ == '__main__':
	node = PersonFollowingNode()
	node.run()