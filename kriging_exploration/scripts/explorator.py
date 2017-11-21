#!/usr/bin/env python


import cv2
import sys

import signal
import numpy as np
import utm


import matplotlib as mpl
import matplotlib.cm as cm

import rospy

import argparse

from cosmos_msgs.msg import KrigInfo

import kriging_exploration.map_coords
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

        self.current_model=-1
        self.draw_mode = 'none'
        self.grid = DataGrid('limits.coords', cell_size)
        
        self.limits_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.limits_canvas.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        
        
        self.grid_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.grid_canvas.draw_grid(self.grid.cells, cell_size, (128,128,128,2), thickness=1)
        
        self.model_canvas=[]
        self.model_canvas_names=[]

        rospy.loginfo("Subscribing to Krig Info")
        rospy.Subscriber("/kriging_data", KrigInfo, self.data_callback)


        self.refresh()

        while(self.running):
            cv2.imshow('image', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)


    def refresh(self):
        self.image = cv2.addWeighted(self.grid_canvas.image, 0.5, self.base_image, 1.0, 0)
        self.image = cv2.addWeighted(self.limits_canvas.image, 0.5, self.image, 1.0, 0)
        if self.draw_mode == "inputs" and self.current_model>=0 :
            self.image = cv2.addWeighted(self.model_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)

    def data_callback(self, msg):
        point_coord = kriging_exploration.map_coords.coord_from_satnav_fix(msg.coordinates)
        for i in msg.data:
            self.grid.add_data_point(i.model_name, point_coord, i.measurement)

        self.vmin, self.vmax = self.grid.get_max_min_vals()
        self.n_models=len(self.grid.models)
        
        for i in self.grid.models:
            if i.name not in self.model_canvas_names:       
                print i.name
                self.model_canvas_names.append(i.name)
                self.model_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
            self.draw_inputs(self.model_canvas_names.index(i.name))


    def draw_inputs(self, nm):
#        font = cv2.FONT_HERSHEY_SIMPLEX
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)
#
        for i in self.grid.models[nm].orig_data:
            cell = self.grid.cells[i.y][i.x]
#                #print i.value
            a= colmap.to_rgba(int(i.value))                
            b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
#                #print a, b
            self.model_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)

#        self.image = self.satellite.base_image.copy()
#        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
#        self.draw_legend(self.vmin, self.vmax)



       
    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False
        elif k == ord('n'):
            print len(self.grid.models)
        elif k == ord('i'):
            self.draw_mode="inputs"
            self.current_model=0
            self.refresh()


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

    
