import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist, Vector3

class TeleopNode():
    def __init__(self):
        rospy.init_node("teleop_node")    
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.magnitude = .5
        self.mappings = {
            'w': Twist(Vector3(magnitude, 0, 0), Vector3(0, 0, 0)),
            's': Twist(Vector3(-1 * magnitude, 0, 0), Vector3(0, 0, 0)),
            'a': Twist(Vector3(0, 0, 0), Vector3(0, 0, -1 * magnitude)),
            'd': Twist(Vector3(0, 0, 0), Vector3(0, 0, magnitude)),
        }

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
            
    def run(self):
        while not rospy_is_shutdown():
            key = self.getKey()
            if key:
                self.vel_pub.publish(self.mapings[key])