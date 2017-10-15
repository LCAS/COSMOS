#!/usr/bin/env python

import signal
import yaml
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


def data_to_yaml(data_fn):
    mods = {}
    mods['names']=['0.0 cm', '2.5 cm' ,'5.0 cm', '7.5 cm', '10.0 cm', '12.5 cm', '15.0 cm', '17.5 cm', '20.0 cm', '22.5 cm' ,'25.0 cm', '27.5 cm', '30.0 cm', '32.5 cm', '35.0 cm', '37.5 cm', '40.0 cm', '42.5 cm', '45.0 cm']
    mods['data']=[]
#    vals=[]
    print "open: " + data_fn
    f = open(data_fn, 'r')
    for line in f:
        line=line.strip('\n')
        a=line.split(';')
        val={}
        val['position']={'lat':float(a[0]), 'lon':float(a[1])}
        val['data']= [float(i) for i in a[2:]]
        mods['data'].append(val)
    
    print mods['data']
    yml = yaml.safe_dump(mods, default_flow_style=False)
    print yml
    filename = data_fn+'.yaml'
    fh = open(filename, "w")
    s_output = str(yml)
    fh.write(s_output)
    fh.close


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
        self.current_model=-1
        self.n_models=0
        
        cv2.namedWindow('simulator')
        cv2.setMouseCallback('simulator', self.click_callback)
        
        while(self.running):
            cv2.imshow('simulator', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)



    def click_callback(self, event, x, y, flags, param):
#        print event, x, y
#        xval = x#(x * self.origin[2]) + self.origin[0]
#        yval = y#-((y * self.origin[2]) - self.origin[1])
#        dists = self.get_distances_to_pose(xval, yval)
#
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            print self.grid.get_cell_inds_from_coords(click_coord)
            self.satellite.draw_coordinate(click_coord,(0,255,0,128))
            self.image = self.satellite.base_image.copy()
#            try:
#                getattr(self, 'cb_' + self.current_mode)(dists, xval, yval)
#            except AttributeError:
#                pass
#
#        if event == cv2.EVENT_LBUTTONUP:
#            if self.current_mode == 'move' and self.moving:
#                self.update_node_position(self.movingnode, xval, yval)
#                # back to move mode
#                self._set_mode('move')
#                self.moving = False


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
            self.grid.load_data_from_yaml('Iains2.yaml')
#            self.grid._load_data('Iains_data0.dat')

            self.vmin, self.vmax = self.grid.get_max_min_vals()
            print "LIMS: " + str(self.vmin) + " " + str(self.vmax)

            self.n_models=len(self.grid.models)
            self.current_model=0
            self.draw_inputs(self.current_model)

        elif k == ord('>'):
            self.current_model+=1
            if self.current_model >= self.n_models:
                self.current_model=0
            self.draw_inputs(self.current_model)
            

        elif k == ord('<'):
            self.current_model-=1
            if self.current_model < 0:
                self.current_model=self.n_models-1
            self.draw_inputs(self.current_model)



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
#        cmap = cm.plasma
#        cmap = cm.gnuplot
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
    
    
    simulator(53.261685, -0.525158, 17, 640, args.cell_size)

