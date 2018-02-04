#!/usr/bin/env python


import cv2
import sys
import yaml

import signal
import numpy as np
#import utm


import matplotlib as mpl
import matplotlib.cm as cm

import rospy

import argparse

import actionlib


from cosmos_msgs.msg import KrigInfo
import kriging_exploration.map_coords
import std_msgs.msg

import open_nav.msg

from kriging_exploration.data_grid import DataGrid
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.visualiser import KrigingVisualiser
from kriging_exploration.canvas import ViewerCanvas
from kriging_exploration.topological_map import TopoMap
from kriging_exploration.exploration import ExplorationPlan

from sensor_msgs.msg import NavSatFix


def overlay_image_alpha(img, img_overlay):
    """Overlay img_overlay on top of img at the position specified by
    pos and blend using alpha_mask.
    """
    show_image = img.copy()
    alpha =  img_overlay[:, :, 3] / 255.0   # Alpha mask must contain values 
                                            # within the range [0, 1] 
                                            # and be the same size as img_overlay.
    # Image ranges
    y1, y2 = 0, img.shape[0]
    x1, x2 = 0, img.shape[1]

    channels = img.shape[2]

    alpha_inv = 1.0 - alpha

    for c in range(channels):
        show_image[y1:y2, x1:x2, c] = (alpha * img_overlay[y1:y2, x1:x2, c] + alpha_inv * img[y1:y2, x1:x2, c])
    return show_image



class Explorator(KrigingVisualiser):

    def __init__(self, lat_deg, lon_deg, zoom, size, args):
        self.targets = []

        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)

        print "Creating visualiser object"
        super(Explorator, self).__init__(lat_deg, lon_deg, zoom, size)

        cv2.namedWindow('explorator')
        cv2.setMouseCallback('explorator', self.click_callback)

        self.current_model=-1
        self.draw_mode = 'none'
        self.grid = DataGrid(args.limits_file, args.cell_size)
        self.topo_map= TopoMap(self.grid)
        self.visited_wp=[]

        self.explo_plan = ExplorationPlan(self.topo_map, args.initial_waypoint, args.initial_percent)
        self.navigating = False
        self.exploring = 0
        
        
        print "NUMBER OF TARGETS:"
        print len(self.explo_plan.targets)        
        
        self.limits_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.grid_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.exploration_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)
        self.gps_canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

        self.limits_canvas.draw_polygon(self.grid.limits, (0,0,255,128), thickness=1)
        self.grid_canvas.draw_grid(self.grid.cells, args.cell_size, (128,128,128,2), thickness=1)
        
        self.redraw()

        self.redraw_kriged=True
        self.redraw_var=True
        self.redraw_devi=True
        
        self.model_canvas=[]
        self.kriging_canvas=[]
        self.klegend_canvas=[]
        self.klegend2_canvas=[]
        self.klegend3_canvas=[]
        self.sigma_canvas=[]
        self.sigma2_canvas=[]
        self.model_canvas_names=[]
        

        rospy.loginfo("Subscribing to Krig Info")
        rospy.Subscriber("/kriging_data", KrigInfo, self.data_callback)
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber('/penetrometer_scan', std_msgs.msg.String, self.scan_callback)
        self.req_data_pub = rospy.Publisher('/request_scan', std_msgs.msg.String, latch=False, queue_size=1)

        rospy.loginfo(" ... Connecting to Open_nav")
        
        self.open_nav_client = actionlib.SimpleActionClient('/open_nav', open_nav.msg.OpenNavAction)
        self.open_nav_client.wait_for_server()

        rospy.loginfo(" ... done")


        tim1 = rospy.Timer(rospy.Duration(0.2), self.drawing_timer_callback)
        tim2 = rospy.Timer(rospy.Duration(0.1), self.control_timer_callback)
        self.refresh()

        while(self.running):
            cv2.imshow('explorator', self.show_image)
            k = cv2.waitKey(20) & 0xFF
            self._change_mode(k)

        tim1.shutdown()
        tim2.shutdown()
        cv2.destroyAllWindows()       
        sys.exit(0)


    def drawing_timer_callback(self, event):
        self.refresh()

    def control_timer_callback(self, event):
        if self.navigating:
            if self.open_nav_client.simple_state ==2:
                print "DONE NAVIGATING"
                self.navigating = False
                if self.exploring:
                   self.explo_plan.explored_wp.append(self.explo_plan.route.pop(0))
                   info_str='Do_reading'
                   self.req_data_pub.publish(info_str)
                   
#        else:
#            if self.exploring:
#                print "waiting for new goal"
            
    def gps_callback(self, data):
        if not np.isnan(data.latitude):
            self.gps_canvas.clear_image()
            gps_coord = MapCoords(data.latitude,data.longitude)            
            self.gps_canvas.draw_coordinate(gps_coord,'black',size=2, thickness=2, alpha=255)
            self.last_coord=gps_coord

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
                self.klegend2_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.klegend3_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))                
                self.sigma_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
                self.sigma2_canvas.append(ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res))
            self.draw_inputs(self.model_canvas_names.index(i.name))


    def scan_callback(self, msg):
        if msg.data == 'Reading':
            print "GOT READING!!!"
            cx, cy = self.grid.get_cell_inds_from_coords(self.last_coord)
            if cx <0 or cy<0:
                print "Reading outside the grid"
            else:
                print 'Reading at: ', cx, cy
                for i in self.topo_map.waypoints:
                    if (cy,cx) == i.ind:
                        print 'Setting: ', i.name, i.coord, "as Visited"
                        i.visited= True
                        self.visited_wp.append(i)
                        self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (0,255,0,2), thickness=1)
                        self.grid_canvas.draw_waypoints(self.visited_wp, (0,0,255,2), thickness=1)
                        self.redraw()


    def refresh(self):
        #self.show_image = self.image.copy()
        #self.show_image = cv2.addWeighted(self.gps_canvas.image, 0.7, self.image, 1.0, 0)
        #self.show_image = transparentOverlay(self.image, self.gps_canvas.image)
        self.show_image = overlay_image_alpha(self.image,self.gps_canvas.image)

    def redraw(self):
        self.image = cv2.addWeighted(self.grid_canvas.image, 0.5, self.base_image, 1.0, 0)
        self.image = cv2.addWeighted(self.limits_canvas.image, 0.75, self.image, 1.0, 0)
        self.image = cv2.addWeighted(self.exploration_canvas.image, 0.75, self.image, 1.0, 0)
        if self.draw_mode == "inputs" and self.current_model>=0 :
            self.image = cv2.addWeighted(self.model_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)

        if self.draw_mode == "kriging":# and self.current_model>=0 :
            self.image = cv2.addWeighted(self.kriging_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.klegend_canvas[self.current_model].image)

        if self.draw_mode == "deviation":# and self.current_model>=0 :
            self.image = cv2.addWeighted(self.sigma_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend3_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.klegend3_canvas[self.current_model].image)
            
        if self.draw_mode == "variance":# and self.current_model>=0 :    
            self.image = cv2.addWeighted(self.sigma2_canvas[self.current_model].image, 0.75, self.image, 1.0, 0)
            #self.image = cv2.addWeighted(self.klegend2_canvas[self.current_model].image, 1.0, self.image, 1.0, 0)
            self.image = overlay_image_alpha(self.image, self.klegend2_canvas[self.current_model].image)
            
        self.show_image = self.image.copy()

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

            

    def click_callback(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_RBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                print cx, cy
                
            for i in self.topo_map.waypoints:
                if (cy,cx) == i.ind:
                    print i.name#, i.coord
                    i.visited= True
                    self.visited_wp.append(i)
                    self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (0,255,0,2), thickness=1)
                    self.grid_canvas.draw_waypoints(self.visited_wp, (0,0,255,2), thickness=1)
                    self.redraw()

        if event == cv2.EVENT_LBUTTONDOWN:
            click_coord = self.satellite._pix2coord(x,y)
            cx, cy = self.grid.get_cell_inds_from_coords(click_coord)

            if cx <0 or cy<0:
                print "click outside the grid"
            else:
                print cx, cy

            for i in self.topo_map.waypoints:
                if (cy,cx) == i.ind:
                    self.open_nav_client.cancel_goal()
                    targ = open_nav.msg.OpenNavActionGoal()

                    #goal.goal.goal.header.
                    targ.goal.coords.header.stamp=rospy.Time.now()
                    targ.goal.coords.latitude=i.coord.lat
                    targ.goal.coords.longitude=i.coord.lon

                    print targ
                    self.navigating=True
                    self.open_nav_client.send_goal(targ.goal)
                    #self.client.wait_for_result()
                    # Prints out the result of executing the action
                    #ps = self.client.get_result()
                    #print ps
                

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
        self.klegend_canvas[nm].draw_legend(self.grid.models[nm].min_val, self.grid.models[nm].max_val, colmap, title="Kriging")
        
        self.redraw()


    def draw_variance(self, nm):
        print "drawing variance" + str(nm)
        norm = mpl.colors.Normalize(vmin=self.grid.models[nm].min_var, vmax=self.grid.models[nm].max_var)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.sigma_canvas[nm].clear_image()
        self.klegend_canvas[nm].clear_image()
        
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].variance[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.sigma2_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        self.klegend2_canvas[nm].put_text(self.grid.models[nm].name)
        self.klegend2_canvas[nm].draw_legend(self.grid.models[nm].min_var, self.grid.models[nm].max_var, colmap, title="Variance")
        self.redraw()


    def draw_deviation(self, nm):
        print "drawing deviation" + str(nm)
        norm = mpl.colors.Normalize(vmin=self.grid.models[nm].min_dev, vmax=self.grid.models[nm].max_dev)
        cmap = cm.jet
        colmap = cm.ScalarMappable(norm=norm, cmap=cmap)

        self.sigma_canvas[nm].clear_image()
        self.klegend3_canvas[nm].clear_image()
        
        for i in range(self.grid.models[nm].shape[0]):
            for j in range(self.grid.models[nm].shape[1]):
                cell = self.grid.cells[i][j]
                a= colmap.to_rgba(int(self.grid.models[nm].deviation[i][j]))
                b= (int(a[2]*255), int(a[1]*255), int(a[0]*255), int(a[3]*50))
                self.sigma_canvas[nm].draw_cell(cell, self.grid.cell_size, b, thickness=-1)


        self.klegend3_canvas[nm].put_text(self.grid.models[nm].name)
        self.klegend3_canvas[nm].draw_legend(self.grid.models[nm].min_dev, self.grid.models[nm].max_dev, colmap, title="Deviation")
        self.redraw()


    def krieg_all_mmodels(self):
        for i in self.grid.models:
            i.do_krigging()
            self.redraw_kriged=True
            self.redraw_var=True
            self.redraw_devi=True

    def draw_all_outputs(self):
        for i in self.grid.models:
            self.draw_krigged(self.model_canvas_names.index(i.name))
            self.redraw_kriged=False


    def draw_all_vars(self):
        for i in self.grid.models:
            self.draw_variance(self.model_canvas_names.index(i.name))
            self.redraw_var=False
    
    def draw_all_devs(self):
        for i in self.grid.models:
            self.draw_deviation(self.model_canvas_names.index(i.name))
            self.redraw_devi=False
        
    
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
        elif k == ord('d'):
            if self.n_models > 0:
                self.draw_mode="deviation"
                self.current_model=0
                if self.redraw_devi:
                    self.draw_all_devs()
                self.redraw()                
        elif k == ord('v'):
            if self.n_models > 0:
                self.draw_mode="variance"
                self.current_model=0
                if self.redraw_var:
                    self.draw_all_vars()
                self.redraw()
        elif k == ord('t'):
            self.krieg_all_mmodels()
            if self.n_models > 0:
                self.draw_all_outputs()
                self.draw_mode="kriging"
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
        elif k == ord('w'):
            self.grid_canvas.draw_waypoints(self.topo_map.waypoints, (0,255,0,2), thickness=1)
            self.grid_canvas.draw_waypoints(self.visited_wp, (0,0,255,2), thickness=1)
            self.redraw()
        elif k == ord('e'):
            self.exploration_canvas.draw_waypoints(self.explo_plan.targets, (255,128,128,2), thickness=2)
            self.exploration_canvas.draw_plan(self.explo_plan.route, (0,128,128,2), thickness=1)
            self.redraw()            
        elif k == ord('g'):
            if len(self.explo_plan.route) >0:
                gg=self.explo_plan.route[0]
                self.open_nav_client.cancel_goal()
                targ = open_nav.msg.OpenNavActionGoal()
    
                targ.goal.coords.header.stamp=rospy.Time.now()
                targ.goal.coords.latitude=gg.coord.lat
                targ.goal.coords.longitude=gg.coord.lon
    
                #print gg
                self.exploring=1
                self.navigating=True
                self.open_nav_client.send_goal(targ.goal)
            else:
                print "Done Exploring"
                self.exploring = 0
        elif k == ord('y'):
            vwp = []
            for i in self.visited_wp:
                vwp.append(i.name)
            yml = yaml.safe_dump(vwp, default_flow_style=False)
            fh = open("visited.yaml", "w")
            s_output = str(yml)
            fh.write(s_output)
            fh.close          
        elif k == ord('l'):
            print "loading visited"
            
            with open("visited.yaml", 'r') as f:
                visited = yaml.load(f)
                for i in visited:
                    for l in self.topo_map.waypoints:
                        if i == l.name:
                            self.visited_wp.append(l)
                            break

    
    def signal_handler(self, signal, frame):
        self.running = False
        print('You pressed Ctrl+C!')





if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--cell_size", type=int, default=10,
                        help="cell size in meters")
    parser.add_argument("--initial_percent", type=float, default=0.05,
                        help="Percentage of cells to be explored on the initial plan") 
    parser.add_argument("--limits_file", type=str, default='limits.coords',
                        help="Percentage of cells to be explored on the initial plan")
    parser.add_argument("--initial_waypoint", type=str, default='WayPoint498',
                        help="Percentage of cells to be explored on the initial plan")
    args = parser.parse_args()
    
    rospy.init_node('kriging_exploration')
    #Explorator(53.261685, -0.527158, 16, 640, args.cell_size)
    
    #Explorator(53.267213, -0.533420, 17, 640, args)  #Football Field
    Explorator(53.261685, -0.525158, 17, 640, args) #COSMOS Field

    
