#!/usr/bin/env python
import time
from geometry_msgs.msg import Twist
import rospy

class SquareDriveNode(object):
	""" This node drives the neato in a square """
	def __init__(self):
		rospy.init_node("square_drive_node")
		# self.vel_msg = Twist()
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.count = 0

	def run(self):		
		while not rospy.is_shutdown():
			vel_msg = Twist()
			vel_msg.linear.x = .25
			vel_msg.linear.y = 0 
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = 0
			self.vel_pub.publish(vel_msg)
			time.sleep(2)
			vel_msg.linear.x = 0
			vel_msg.angular.z = 1
			self.vel_pub.publish(vel_msg)
			time.sleep(1.6)
			self.count += 1
			if self.count > 4:
				vel_msg.linear.x  = 0
				vel_msg.angular.z = 0
				self.vel_pub.publish(vel_msg)
				break


if __name__ == '__main__':
	node = SquareDriveNode()
	node.run()