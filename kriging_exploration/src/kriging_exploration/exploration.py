import utm
import yaml
import random

import numpy as np

from map_coords import MapCoords

class ExplorationPlan(object):
    def __init__(self, topo_map, initial_waypoint):
        self.targets=[]
        self.route=[]
        self.route_nodes=[]
        self._generate_targets(topo_map, initial_waypoint)
        self._get_plan(initial_waypoint)
        
        
    def _get_wp(self, wp):
        for i in self.targets:
            if i.name == wp:
                return i
                
    
    def _generate_targets(self, topo_map, initial_waypoint):
        ntargets = int(np.ceil(len(topo_map.waypoints))*0.05)
        N = 0
        for item in topo_map.waypoints:
            N += 1
            if len( self.targets ) < ntargets:
                self.targets.append(item)
            else:
                s = int(random.random() * N)
                if s < ntargets:
                    self.targets[ s ] = item

        found = False            
        for i in self.targets:
            if i.name == initial_waypoint:
                found=True
                break
        
        if not found:
            for i in topo_map.waypoints:
                if i.name == initial_waypoint:
                    self.targets.append(i)
                    break
        
    def _get_plan(self, initial_waypoint):
        self.route.append(self._get_wp(initial_waypoint))
        self.route_nodes.append(initial_waypoint)
        
        self._create_greedy_plan()
        #self._create_random_plan(initial_waypoint)
        
    
    def _create_greedy_plan(self):
        #got_plan = False
        local_targs = self.targets[:]
        
        while len(local_targs) > 0:
            print "current node: " + self.route_nodes[-1]
            cn = self.route[-1]
            min_dist =10000.0
            min_ind = -1
            for i in range(0, len(local_targs)):
                dist, ang = local_targs[i].coord - cn.coord
                if dist <= min_dist:
                   min_dist = dist
                   min_ind = i
            self.route.append(local_targs[min_ind])
            self.route_nodes.append(local_targs[min_ind].name)
            local_targs.pop(min_ind)
        
        print self.route_nodes
        print [x.name for x in self.route]
        print len(self.route_nodes)
            
            
        
    
    def _create_random_plan(self, initial_waypoint):
        for i in self.targets:
            if i.name != initial_waypoint:
                self.route.append(i)
                self.route_nodes.append(i.name)
        