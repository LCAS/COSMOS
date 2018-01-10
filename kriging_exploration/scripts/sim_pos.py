#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from kriging_exploration.map_coords import MapCoords
#import time
from sensor_msgs.msg import NavSatFix

class SImPos(object):

    def __init__(self, lat, lon, northang):
        self.gps_coord = MapCoords(lat,lon)
        self.northang = northang
        self.cmd = Twist()

        self.timer= rospy.Timer(rospy.Duration(0.5), self.time_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("/fix", NavSatFix)
        rospy.spin()
        self.timer.shutdown()

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        self.cmd = data
    
    def time_callback(self, event):
        self.precord = self.gps_coord
        self.northang = self.northang + (self.cmd.angular.x/0.2)
        if self.northang > math.pi:
            self.northang = self.northang - math.pi
        if self.northang < -math.pi:
            self.northang = self.northang + math.pi
        northing_dif = self.cmd.linear.x * math.sin(self.northang)
        easting_dif =  self.cmd.linear.x * math.cos(self.northang)
        self.gps_coord = self.gps_coord._get_rel_point(easting_dif, northing_dif)
        print self.northang
        print self.gps_coord
        fix = NavSatFix()
        fix.latitude = self.gps_coord.lat
        fix.longitude = self.gps_coord.lon
        self.pub.publish(fix)


if __name__ == '__main__':
    rospy.init_node('sim_pos')   
    SImPos(53.261685, -0.525158, 0.0)


