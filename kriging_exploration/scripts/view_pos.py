#!/usr/bin/env python


import cv2
import sys

import signal
import numpy as np

import rospy

from sensor_msgs.msg import NavSatFix


#import argparse
#
#import matplotlib as mpl
#import matplotlib.cm as cm

#import satellite
#from kriging_exploration.satellite import SatelliteImage
#from kriging_exploration.data_grid import DataGrid
#from krigging_data import KriggingData


from kriging_exploration.map_coords import MapCoords
from kriging_exploration.visualiser import KrigingVisualiser


class SimpleDataVisualiser(KrigingVisualiser):

    def __init__(self, zoom, size, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        initial_pose = rospy.wait_for_message("/fix", NavSatFix, 10)
        print initial_pose


        super(SimpleDataVisualiser, self).__init__(initial_pose.latitude, initial_pose.longitude, zoom, size)
        
        rospy.loginfo("Subscribing to GPS Data")
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        
        while(self.running):
            cv2.imshow('image', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)


    def gps_callback(self, data):
        self.draw_coordinate(data.latitude, data.longitude,'white',size=2, thickness=1)

    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False


    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)


if __name__ == '__main__':
#    parser = argparse.ArgumentParser()
#    parser.add_argument("--cell_size", type=int, default=10,
#                        help="cell size in meters")
#    args = parser.parse_args()
    
    rospy.init_node('kriging_exploration')
    SimpleDataVisualiser(17, 640, 10)
