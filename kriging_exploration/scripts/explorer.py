#!/usr/bin/env python

import signal
import yaml
import numpy as np

import cv2
import sys

import argparse

import matplotlib as mpl
import matplotlib.cm as cm

import rospy

import std_msgs

from cosmos_msgs.msg import KrigInfo
#from cosmos_msgs.msg import KrigMsg


import kriging_exploration.map_coords
from kriging_exploration.satellite import SatelliteImage
#from kriging_exploration.map_coords import MapCoords
from kriging_exploration.data_grid import DataGrid
#from krigging_data import KriggingData

#
#def coord_from_satnav_fix(msg):
#    a = MapCoords(msg.latitude, msg.longitude)
#    return a


class explorer(object):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, lat_deg, lon_deg, zoom, size, cell_size):
        self.draw_mode = 'none'
        self.running = True
        self.current_model=-1
        self.n_models=0
        
        cv2.namedWindow('explorer')

        
        signal.signal(signal.SIGINT, self.signal_handler)


        print "Loading Satellite Image"
        self.satellite = SatelliteImage(lat_deg, lon_deg, zoom, size)
        self.grid = DataGrid('limits.coords', cell_size)


        rospy.loginfo("Subscribing to Krig Info")
        rospy.Subscriber("/kriging_data", KrigInfo, self.data_callback)

        
        self.image = self.satellite.base_image.copy()
        while(self.running):
            cv2.imshow('explorer', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)


    def data_callback(self, msg):
        point_coord = kriging_exploration.map_coords.coord_from_satnav_fix(msg.coordinates)
        for i in msg.data:
            self.grid.add_data_point(i.model_name, point_coord, i.measurement)

        self.vmin, self.vmax = self.grid.get_max_min_vals()
        self.n_models=len(self.grid.models)

    def krieg_all_mmodels(self):
        for i in self.grid.models:
            i.do_krigging()



    def _change_mode(self, k):
        if k == 27:           
            self.running = False
            
        elif k == ord('q'):
            self.running = False
            
        elif k == ord('c'):
            print "Draw corners"
            self.satellite.draw_coordinate(self.grid.swc,(0,255,0,128))
            self.satellite.draw_coordinate(self.grid.sec,(0,0,255,128))
            self.satellite.draw_coordinate(self.grid.nwc,(255,0,0,128))
            self.satellite.draw_coordinate(self.grid.nec,(255,255,255,128))
            self.image = self.satellite.base_image.copy()

        elif k == ord('i'):
            self.draw_mode='inputs'
            self.draw_inputs(self.current_model)

        elif k == ord('t'):
            self.krieg_all_mmodels()
            self.draw_mode='krigging'
            self.draw_krigged(self.current_model)

        elif k == ord('v'):
            self.draw_mode='sigma'
            self.draw_deviation(self.current_model)

        elif k == ord('>'):
            self.current_model+=1
            if self.current_model >= self.n_models:
                self.current_model=0

            if self.draw_mode=='inputs':
                self.draw_inputs(self.current_model)
            if  self.draw_mode=='krigging':
                self.draw_krigged(self.current_model)
            if  self.draw_mode=='sigma':
                self.draw_deviation(self.current_model)          

        elif k == ord('<'):
            self.current_model-=1
            if self.current_model < 0:
                self.current_model=self.n_models-1

            if self.draw_mode=='inputs':
                self.draw_inputs(self.current_model)
            if  self.draw_mode=='krigging':
                self.draw_krigged(self.current_model)
            if  self.draw_mode=='sigma':
                self.draw_deviation(self.current_model)

        elif k == ord('l'):
            print "Draw Limits"
            #self.satellite.draw_list_of_coords(self.kriged.limits, (0,0,255,128))
            self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
            self.image = self.satellite.base_image.copy()
        elif k == ord('g'):
            print "Draw Grid"
            print "Grid Shape: ", self.grid.shape
            self.satellite.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)            
#            print len(self.kriged.waypoints), len(self.kriged.waypoints[0])
#            for i in range(0, len(self.kriged.waypoints)):
#                self.satellite.draw_list_of_coords(self.kriged.waypoints[i], (128,0,128,64), size=2, thickness=-1)
            self.image = self.satellite.base_image.copy()




    def draw_deviation(self, nm):
        font = cv2.FONT_HERSHEY_SIMPLEX
        norm = mpl.colors.Normalize(vmin=0, vmax=100)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

#        vmin = np.floor(self.grid.data[0].lims[0])
#        vmax = np.ceil(self.grid.data[0].lims[1])
#        norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#        cmap = cm.jet
#        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
                   
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].sigmapercent[i][j]*100))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)

        self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.image = self.satellite.base_image.copy()        
        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        self.draw_legend(self.vmin, self.vmax)




    def draw_krigged(self, nm):
        font = cv2.FONT_HERSHEY_SIMPLEX
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

#        vmin = np.floor(self.grid.data[0].lims[0])
#        vmax = np.ceil(self.grid.data[0].lims[1])
#        norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#        cmap = cm.jet
#        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
                   
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].output[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)

        self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.image = self.satellite.base_image.copy()        
        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        self.draw_legend(self.vmin, self.vmax)



    def draw_inputs(self, nm):
#        vmin = np.floor(self.grid.models[nm].lims[0])
#        vmax = np.ceil(self.grid.models[nm].lims[1])
#        print vmin, vmax            
#
        font = cv2.FONT_HERSHEY_SIMPLEX
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
            self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)

        self.image = self.satellite.base_image.copy()
        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        self.draw_legend(self.vmin, self.vmax)
    
    
    def draw_legend(self, vmin, vmax):
        step = (vmax - vmin)/(600-40)
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        vp = range(int(np.floor(vmin)),int(np.ceil(vmax)), int(np.ceil(step)))
        print len(vp)        

        if step>1.0:
            ind = 0
            while ind < 560:
#                print int(vmin+(ind*step))
                a= colmap.to_rgba(int(vmin+(ind*step)))                
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))                
                cv2.rectangle(self.image, (int(ind+40), int(580)), (int(ind+1+40), int(600)), b , thickness=-1)
                ind+=1
#            cv2.rectangle(self.image, (int(40), int(580)), (int(600), int(600)), (200,0,0,0), thickness=-1)


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
    explorer(53.261685, -0.525158, 17, 640, args.cell_size)

