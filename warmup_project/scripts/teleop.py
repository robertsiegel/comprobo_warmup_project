#!/usr/bin/env python
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

class TeleopNode(object):
    def __init__(self):
        rospy.init_node("teleop_node")    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.magnitude = .5
        self.mappings = {
            'w': Twist(Vector3(self.magnitude, 0, 0), Vector3(0, 0, 0)),
            's': Twist(Vector3(-1 * self.magnitude, 0, 0), Vector3(0, 0, 0)),
            'a': Twist(Vector3(0, 0, 0), Vector3(0, 0, self.magnitude)),
            'd': Twist(Vector3(0, 0, 0), Vector3(0, 0, -1 * self.magnitude)),
        }

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
            
    def run(self):
        while not rospy.is_shutdown():
            key = self.getKey()
            if key and key in self.mappings:
                self.vel_pub.publish(self.mappings[key])
            else:
                self.vel_pub.publish(Twist())
                if key == '\x03':
                    break

if __name__ == '__main__':
    TeleopNode().run()