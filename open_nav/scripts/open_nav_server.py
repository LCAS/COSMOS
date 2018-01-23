#! /usr/bin/env python

import numpy as np
import rospy

import actionlib
from sensor_msgs.msg import NavSatFix
import open_nav.msg

from geometry_msgs.msg import Twist

from kriging_exploration.map_coords import MapCoords

class opennavserver(object):

    _feedback = open_nav.msg.OpenNavActionFeedback()
    _result   = open_nav.msg.OpenNavResult()

    def __init__(self, name):
        self.cancelled = False
        self._action_name = name
        self.first_fix = True
        
        #self.brand_image_path = rospy.get_param("~brand_image_path",'/tmp/Tweeter_branding.png')
        rospy.loginfo("Creating action servers.")
        print self._action_name
        self._as = actionlib.SimpleActionServer(self._action_name, open_nav.msg.OpenNavAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)

        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)

        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Ready ...")

        self.pub = rospy.Publisher('/cmd_vel', Twist)

        rospy.spin()


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_coord = MapCoords(data.latitude,data.longitude)
            if not self.first_fix:
                ang = self.gps_coord - self.last_coord
                print ang
            self.first_fix = False
            self.last_coord=self.gps_coord
            
            #self.refresh()
        
    def executeCallback(self, goal):
        rospy.loginfo("Navigating...")
        #open_nav.msg.OpenNavActionFeedback.feedback.progress
        #self._feedback.feedback.progress = 'Navigating...'
        #self._as.publish_feedback(self._feedback)

        print goal

        goal_coord = MapCoords(goal.coords.latitude,goal.coords.longitude)

        print "RESULT"
        print self.gps_coord - goal_coord

        result = True
        
        if not self.cancelled :
            self._result.success = result
            #self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)



    def preemptCallback(self):
        self.cancelled = True
        self._result.success = False
        self._as.set_preempted(self._result)



if __name__ == '__main__':
    rospy.init_node('open_nav')
    server = opennavserver(rospy.get_name())