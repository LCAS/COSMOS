#!/usr/bin/env python

# license removed for brevity


import rospy
from std_msgs.msg import String
import time
from auto_soil_probe.msg import Controller

import matplotlib.pyplot as plt
import numpy as np




class get_penetromter_data(object):

    def __init__(self) :
        self.n60s=0
        self.wpn=0
        self.wpfdata=[]
        self.wptdata=[]
        filename= "Waypoint"+str(self.wpn)+".log"
        rospy.on_shutdown(self.on_shutdown)
        self.fileo = open(filename, 'w+')
        rospy.Subscriber("/auto_soil_probe/data", Controller, self.data_callback)
        rospy.spin()
        #self.fileo.close()

    def data_callback(self, msg):
        force= msg.z_axis_force
        targ = msg.z_axis_target_position
        
        if targ <= 400.0:
            self.n60s+=1
        else:
            self.n60s=0
        
        if self.n60s < 250:
            self.wpfdata.append(force)
            self.wptdata.append(msg.header.stamp.secs)
            txt = str(msg.header.stamp.secs) + ', ' + str(force) + ', ' + str(msg.z_axis_target_position) + ', ' + str(msg.z_axis_speed) + '\n'
            print txt
            self.fileo.write(txt)
           
        if self.n60s == 250:
            self.create_graph()
            self.wpn+=1
            self.fileo.close()
            filename= "Waypoint"+str(self.wpn)+".log"
            self.fileo = open(filename, 'w+')
            self.wpfdata=[]
            self.wptdata=[]
            print filename
        
    def create_graph(self):
        plt.plot(self.wptdata, self.wpfdata)
        plt.xlabel('time (s)')
        plt.ylabel('current')
        title="Waypoint"+str(self.wpn)+".png"
        plt.title(title)
        plt.grid(True)
        plt.savefig(title)
        plt.clf()
    
    def on_shutdown(self):
        self.create_graph()
        self.fileo.close()
    
if __name__ == '__main__':
    rospy.init_node('data_logger')
    server = get_penetromter_data()
