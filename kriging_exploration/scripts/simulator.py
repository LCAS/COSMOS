#!/usr/bin/env python

import signal
import numpy as np

import cv2
import sys

import argparse

import matplotlib as mpl
import matplotlib.cm as cm

#import satellite
from kriging_exploration.satellite import SatelliteImage
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.data_grid import DataGrid
#from krigging_data import KriggingData


def load_data(data_fn):
#    data=[]
#    vals=[]
    print "open: " + data_fn
    f = open(data_fn, 'r')
    for line in f:
        line=line.strip('\n')
        a=line.split(';')
        print a
        b=MapCoords(float(a[0]),float(a[1]))
        print b
        
#        cx = int(np.floor((b.easting - self.swc.easting)/self.cell_size))
#        cy = int(np.floor(((b.northing - self.swc.northing)/self.cell_size)))
#        data.append(KriggingDataPoint(b,(cx,cy),float(a[2])))
#        vals.append(float(a[2]))

    
#    lims= [np.min(vals), np.max(vals)]
#    a = KriggingData(self.shape)
#    a.add_data(data, lims)
#    self.data.append(a)
    #print len(self.data_coords), self.data_coords[0]


class simulator(object):

    modes = {
        'help': 'press \'h\' for help',
        'none': ''
    }

    def __init__(self, lat_deg, lon_deg, zoom, size, cell_size):
        
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        print "Loading Satellite Image"
        self.satellite = SatelliteImage(lat_deg, lon_deg, zoom, size)


        self.grid = DataGrid('limits.coords', cell_size)
        
        self.image = self.satellite.base_image.copy()
        
        
        while(self.running):
            cv2.imshow('image', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)


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
            
        elif k == ord('t'):
            print "t"
#            print "DATA"
#            self.grid.data[0].do_krigging()
#
#            vmin = np.floor(self.grid.data[0].lims[0])
#            vmax = np.ceil(self.grid.data[0].lims[1])
#            print vmin, vmax            
#            
#            norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#            cmap = cm.jet
#            colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
#                       
#            for i in range(self.grid.data[0].shape[0]):
#                for j in range(self.grid.data[0].shape[1]):
#                    cell = self.grid.cells[i][j]
#                    a= colmap.to_rgba(int(self.grid.data[0].output[i][j]))
#                    b= (int(a[0]*255), int(a[1]*255), int(a[2]*255), int(a[3]*50))
#                    self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)
#
#            self.satellite.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
#            #self.satellite.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)
#            self.image = self.satellite.base_image.copy()

        elif k == ord('v'):
            print "v"
#            vmin = np.floor(self.grid.data[0].min_var*100)
#            vmax = np.ceil(self.grid.data[0].max_var*100)
#            print vmin, vmax            
#            
#            norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#            cmap = cm.jet
#            colmap = cm.ScalarMappable(norm=norm, cmap=cmap)            
#                       
#            for i in range(self.grid.data[0].shape[0]):
#                for j in range(self.grid.data[0].shape[1]):
#                    cell = self.grid.cells[i][j]
#                    a= colmap.to_rgba(int((1-self.grid.data[0].variance[i][j])*100))
#                    b= (int(a[0]*255), int(a[1]*255), int(a[2]*255), int(a[3]*50))
#                    self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)
#
#            self.satellite.draw_polygon(self.grid.limits, (255,255,255,128), thickness=1)
#            #self.satellite.draw_grid(self.grid.cells, self.grid.cell_size, (64,64,64,64), thickness=1)
#            self.image = self.satellite.base_image.copy()

        elif k == ord('i'):
            print "TEST"
            load_data('Iains2.csv')
#            self.grid._load_data('Iains_data0.dat')
#            print "LIMS:"
#            vmin = np.floor(self.grid.data[0].lims[0])
#            vmax = np.ceil(self.grid.data[0].lims[1])
#            print vmin, vmax            
#            
#            norm = mpl.colors.Normalize(vmin=vmin, vmax=vmax)
#            cmap = cm.jet
#            colmap = cm.ScalarMappable(norm=norm, cmap=cmap)
#
#            for i in self.grid.data[0].orig_data:
#                cell = self.grid.cells[i.y][i.x]
#                #print i.value
#                a= colmap.to_rgba(int(i.value))                
#                b= (int(a[0]*255), int(a[1]*255), int(a[2]*255), int(a[3]*50))
#                #print a, b
#                self.satellite.draw_cell(cell, self.grid.cell_size, b, thickness=-1)
#            self.image = self.satellite.base_image.copy()                



    def signal_handler(self, signal, frame):
        cv2.destroyAllWindows()
        print('You pressed Ctrl+C!')
        sys.exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    
    
    simulator(53.261685, -0.525158, 17, 640, args.cell_size)

