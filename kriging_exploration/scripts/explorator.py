#!/usr/bin/env python


import cv2
import sys

import signal
import numpy as np
#import utm


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
from kriging_exploration.topological_map import TopoMap
from kriging_exploration.exploration import ExplorationPlan

from sensor_msgs.msg import NavSatFix

class Explorator(KrigingVisualiser):


    def __init__(self, lat_deg, lon_deg, zoom, size, cell_size):
        self.targets = []


        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        print "Creating visualiser object"
        super(Explorator, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('explorator')
        cv2.setMouseCallback('explorator', self.click_callback)

        self.current_model=-1
        self.draw_mode = 'none'
        self.grid = DataGrid('limits.coords', cell_size)
        self.topo_map= TopoMap(self.grid)
        self.explo_plan = ExplorationPlan(self.topo_map, 'WayPoint498')
        
        
        print "NUMBER OF TARGETS:"
        print len(self.explo_plan.targets)        
        
        self.limits_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.grid_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.exploration_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

        self.limits_canvas.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.grid_canvas.draw_grid(self.grid.cells, cell_size, (128,128,128,2), thickness=1)
        
        self.redraw()
        
        self.model_canvas=[]
        self.kriging_canvas=[]
        self.klegend_canvas=[]
        self.sigma_canvas=[]
        self.model_canvas_names=[]
        

        rospy.loginfo("Subscribing to Krig Info")
        rospy.Subscriber("/kriging_data", KrigInfo, self.data_callback)
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)

        tim1 = rospy.Timer(rospy.Duration(0.2), self.timer_callback)
        self.refresh()

        while(self.running):
            cv2.imshow('explorator', self.image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)

        tim1.shutdown()
        cv2.destroyAllWindows()       
        sys.exit(0)


    def timer_callback(self, event):
#        print 'Timer called at ' + str(event.current_real)
        self.refresh()

    def redraw(self):
        self.image = cv2.addWeighted(self.grid_canvas.image, 0.5, self.base_image, 1.0, 0)
        self.image = cv2.addWeighted(self.limits_canvas.image, 0.5, self.image, 1.0, 0)
        self.image = cv2.addWeighted(self.exploration_canvas.image, 0.75, self.image, 1.0, 0)
        if self.draw_mode == "inputs" and self.current_model>=0 :
            self.image = cv2.addWeighted(self.model_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
        if self.draw_mode == "kriging":# and self.current_model>=0 :
            self.image = cv2.addWeighted(self.kriging_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            self.image = cv2.addWeighted(self.klegend_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)

        

    def refresh(self):
        self.image = cv2.addWeighted(self.gps_canvas.image, 0.25, self.image, 1.0, 0)


    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            gps_coord = MapCoords(data.latitude,data.longitude)
            self.gps_canvas.draw_coordinate(gps_coord,'white',size=2, thickness=1, alpha=125)
            #self.refresh()

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
                self.kriging_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.klegend_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.sigma_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
            self.draw_inputs(self.model_canvas_names.index(i.name))


    def draw_inputs(self, nm):
        
        norm = mpl.colors.Normalize(vmin=self.vmin, vmax=self.vmax)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.model_canvas[nm].clear_image()
        for i in self.grid.models[nm].orig_data:
            cell = self.grid.cells[i.y][i.x]
            a= colmap.to_rgba(int(i.value))                
            b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
            self.model_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)
            self.model_canvas[nm].put_text(self.grid.models[nm].name)
#        self.image = self.satellite.base_image.copy()
#        cv2.putText(self.image, self.grid.models[nm].name, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
#        self.draw_legend(self.vmin, self.vmax)

    def click_callback(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                print cx, cy
                
            for i in self.topo_map.waypoints:
                if (cy,cx) == i.ind:
                    print i.name
                

    def draw_krigged(self, nm):
        print "drawing kriging" + str(nm)
        norm = mpl.colors.Normalize(vmin=self.grid.models[nm].min_val, vmax=self.grid.models[nm].max_val)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)


        self.kriging_canvas[nm].clear_image()
        self.klegend_canvas[nm].clear_image()
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].output[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))    
                self.kriging_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)
        
        self.klegend_canvas[nm].put_text(self.grid.models[nm].name)
        self.klegend_canvas[nm].draw_legend(self.grid.models[nm].min_val, self.grid.models[nm].max_val, colmap)
        
        self.redraw()


    def draw_deviation(self, nm):

        norm = mpl.colors.Normalize(vmin=0, vmax=100)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.sigma_canvas[nm].clear_image()
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].sigmapercent[i][j]*100))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.sigma_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        self.sigma_canvas[nm].put_text(self.grid.models[nm].name)
        self.redraw()


    def krieg_all_mmodels(self):
        for i in self.grid.models:
            i.do_krigging()
            self.draw_krigged(self.model_canvas_names.index(i.name))

       
    def _change_mode(self, k):
        if k == 27:
            self.running = False
        elif k == ord('q'):
            self.running = False
        elif k == ord('n'):
            print len(self.grid.models)
        elif k == ord('i'):
            if self.n_models > 0:
                self.draw_mode="inputs"
                self.current_model=0
                self.redraw()
        elif k == ord('v'):
            if self.n_models > 0:
                self.draw_mode="variance"
                self.current_model=0
                self.redraw()
        elif k == ord('>'):
            self.current_model+=1
            if self.current_model >= self.n_models:
                self.current_model=0
            self.redraw()
        elif k == ord('<'):
            self.current_model-=1
            if self.current_model < 0:
                self.current_model=self.n_models-1
            self.redraw()
        elif k == ord('t'):
            self.krieg_all_mmodels()
            if self.n_models > 0:
                self.draw_mode="kriging"
                self.current_model=0
                self.redraw()
        elif k == ord('w'):
            self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (128,128,128,2), thickness=1)
            self.redraw()
        elif k == ord('e'):
            self.exploration_canvas.draw_waypoints(self.explo_plan.targets, (255,128,128,2), thickness=2)
            self.exploration_canvas.draw_plan(self.explo_plan.route, (0,128,128,2), thickness=1)
            self.redraw()            
    

        
    
    
    def signal_handler(self, signal, frame):
        self.running = False
        print('You pressed Ctrl+C!')





if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    args = parser.parse_args()
    
    rospy.init_node('kriging_exploration')
    #Explorator(53.261685, -0.527158, 16, 640, args.cell_size)
    Explorator(53.261685, -0.525158, 17, 640, args.cell_size)

    
