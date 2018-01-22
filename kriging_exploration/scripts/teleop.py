#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
import serial
#import time


class OpenNav(object):

    def __init__(self):
        rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.spin()

    def callback(self, data):
        cmd = Twist()
        if data.buttons[4]:
            velx=data.axes[1]*1.0
            vela=data.axes[0]*1.0
            cmd.linear.x=velx
            cmd.angular.z=vela
            self.pub.publish(cmd)
            print cmd
        else:
            self.pub.publish(cmd)
            print cmd



if __name__ == '__main__':
    rospy.init_node('teleop')   
    OpenNav()


