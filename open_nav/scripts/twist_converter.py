#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from base_drive_chain.msg import ThorvaldTwist
#import time



class MessageConverter(object):

    def __init__(self):
        

        rospy.Subscriber("/cmd_vel_thorval", Twist, self.callback)
        self.pub = rospy.Publisher("/cmd_vel", ThorvaldTwist)
        rospy.spin()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        #print data
        cmd = ThorvaldTwist()
        cmd.drive_mode=0
        
        cmd.twist.twist.twist = data
        self.pub.publish(cmd)



if __name__ == '__main__':
    rospy.init_node('message_converter')
    MessageConverter()


