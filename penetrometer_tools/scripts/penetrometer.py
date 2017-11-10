#!/usr/bin/env python

import sys
import rospy

import std_msgs.msg
import sensor_msgs.msg
from geometry_msgs.msg import Pose




class take_scan(object):

    def __init__(self) :
        self.take_scan=False
        self.send_home=False
        self.move_n_metres=False
        rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.JoyCallback)      
        self.cmd_pub = rospy.Publisher('auto_soil_probe/asp_cmd', std_msgs.msg.String, latch=True, queue_size=1)
        
        rospy.loginfo("All Done ...")
        rospy.spin()


    def JoyCallback(self, msg) :
        if msg.buttons[0]:
            if not self.move_n_metres:
                self.move_n_metres=True
                self.move()
        if msg.buttons[2]:
            if not self.send_home:
                self.send_home=True
                self.goto_home()
        if msg.buttons[3]:
            if not self.take_scan:
                self.take_scan=True
                self.take_reading()
        
    def goto_home(self):
        cmd_str='c'
        self.cmd_pub.publish(cmd_str)
        rospy.sleep(2)
        self.send_home =False
        
    
    def take_reading(self):
        cmd_str='p'
        self.cmd_pub.publish(cmd_str)
        rospy.sleep(10)
        self.take_scan=False
        
        

    def move(self):
        print "I will move here"
        self.move_n_metres=False

if __name__ == '__main__':
    rospy.init_node('take_scan')
    server = take_scan()