#!/usr/bin/env python


import cv2
import sys

import signal
import numpy as np
import utm

import rospy

import argparse


from kriging_exploration.data_grid import DataGrid
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.canvas import ViewerCanvas

class Explorator(KrigingVisualiser):


    def __init__(self, lat_deg, lon_deg, zoom, size, cell_size):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        print "Creating visualiser object"
        super(Explorator, self).__init__(lat_deg, lon_deg, zoom, size)

        self.grid = DataGrid('limits.coords', cell_size)
        self.limits_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)        
        self.limits_canvas.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.image = cv2.addWeighted(self.limits_canvas.image, 0.5, self.base_image, 0.9, 0)

        while(self.running):
            cv2.imshow('image', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)


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
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    
    rospy.init_node('kriging_exploration')
    Explorator(53.261685, -0.525158, 17, 640, args.cell_size)

    
