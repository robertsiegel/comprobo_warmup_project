#!/usr/bin/env python
from collections import deque
from teleop import TeleopNode
from person_follower import PersonFollowingNode
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import rospy
import tty
import select
import sys
import termios
import time

class StateControlNode(object):
    def __init__(self):
        rospy.init_node("state_control_node")
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.state = 0
        self.last_time = time.time()
        self.magnitude = .5
        self.mappings = {
            'w': Twist(Vector3(self.magnitude, 0, 0), Vector3(0, 0, 0)),
            's': Twist(Vector3(-1 * self.magnitude, 0, 0), Vector3(0, 0, 0)),
            'a': Twist(Vector3(0, 0, 0), Vector3(0, 0, self.magnitude)),
            'd': Twist(Vector3(0, 0, 0), Vector3(0, 0, -1 * self.magnitude)),
        }
        self.person_angle_range = None
        self.person_distance = None
        self.noise_tolerance = .2
        self.distance_tolerance = .2
        self.adjust_tolerance = 20

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

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
            if self.state:
                key = self.getKey()
                if key and key in self.mappings:
                    self.vel_pub.publish(self.mappings[key])
                else:
                    self.vel_pub.publish(Twist())
                    if key == '\x03':
                        break
            else:
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
            if time.time() - self.last_time >= 10:
                self.state ^= 1
                self.last_time = time.time()

if __name__ == '__main__':
    StateControlNode().run()
