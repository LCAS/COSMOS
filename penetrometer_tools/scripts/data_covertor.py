#!/usr/bin/env python

# license removed for brevity

import yaml

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import time
from auto_soil_probe.msg import Controller

import matplotlib.pyplot as plt
import numpy as np




class get_penetromter_data(object):

    def __init__(self) :
        self.scanning = False
        self.grounded = False
        self.default_force = 0.0
        self.data_out={}
#        self.n60s=0
#        self.wpn=0
#        self.wpfdata=[]
#        self.wptdata=[]
#        filename= "Waypoint"+str(self.wpn)+".log"
        rospy.on_shutdown(self.on_shutdown)
        #self.fileo = open(filename, 'w+')
        rospy.Subscriber("/auto_soil_probe/data", Controller, self.data_callback)
        rospy.Subscriber("/penetrometer/force", Float32, self.force_data_callback)
        rospy.Subscriber("/navsat_fix", NavSatFix, self.gps_data_callback)
        rospy.spin()

        #self.fileo.close()


    def gps_data_callback(self, msg):
        self.gps_pos = msg

    def force_data_callback(self, msg):
        if self.scanning:
            if not self.grounded:
                if self.zpos < 50:
                    self.default_force = msg.data
                elif msg.data >=0.70:
                    self.grounded = True
                    data={}
                    data["z"]=self.zpos
                    data["force"] = msg.data
                    self.data_out["data"].append(data)
                    print data
            else:
                if msg.data > 0:
                    data={}
                    data["z"]=self.zpos
                    data["force"] = msg.data
                    self.data_out["data"].append(data)
                    print data
                else:
                    self.grounded = False

    def data_callback(self, msg):
#        force= msg.z_axis_force
#        targ = msg.z_axis_target_position
        #print msg
        if not self.scanning:
            if msg.position.z >14.0 and self.pre_msg.position.z <= 14:
                print "scanning"
                self.data_out['stamp'] = msg.header.stamp.secs
                self.data_out['fix']={}
                self.data_out['fix']['lat'] = self.gps_pos.latitude
                self.data_out['fix']['lon'] = self.gps_pos.longitude
                self.data_out['fix']['alt'] = self.gps_pos.altitude
                self.data_out['data'] = []
                self.scanning = True
                self.zpos = msg.position.z
        else:
            self.zpos = msg.position.z
            if msg.position.z <= 14.0:
                print "scan end"
                self.scanning = False
                print self.data_out
    
                outfile = str(self.data_out['stamp'])+'.yaml'            
                yml = yaml.safe_dump(self.data_out, default_flow_style=False)
                fh = open(outfile, "w")
                s_output = str(yml)
                print s_output
                fh.write(s_output)
                fh.close
            
            
            
            
        self.pre_msg = msg
#        if targ <= 400.0:
#            self.n60s+=1
#        else:
#            self.n60s=0
#        
#        if self.n60s < 250:
#            self.wpfdata.append(force)
#            self.wptdata.append(msg.header.stamp.secs)
#            txt = str(msg.header.stamp.secs) + ', ' + str(force) + ', ' + str(msg.z_axis_target_position) + ', ' + str(msg.z_axis_speed) + '\n'
#            print txt
#            self.fileo.write(txt)
#           
#        if self.n60s == 250:
#            self.create_graph()
#            self.wpn+=1
#            self.fileo.close()
#            filename= "Waypoint"+str(self.wpn)+".log"
#            self.fileo = open(filename, 'w+')
#            self.wpfdata=[]
#            self.wptdata=[]
#            print filename
        
#    def create_graph(self):
#        plt.plot(self.wptdata, self.wpfdata)
#        plt.xlabel('time (s)')
#        plt.ylabel('current')
#        title="Waypoint"+str(self.wpn)+".png"
#        plt.title(title)
#        plt.grid(True)
#        plt.savefig(title)
#        plt.clf()
    
    def on_shutdown(self):
        print "BYE"
#        self.create_graph()
#        self.fileo.close()
    
if __name__ == '__main__':
    rospy.init_node('data_logger')
    server = get_penetromter_data()
