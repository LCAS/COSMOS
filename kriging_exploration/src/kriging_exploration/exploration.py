import utm
import yaml
import random

import numpy as np

from map_coords import MapCoords



#def line_intersection(line1, line2):
#    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
#    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here
#
#    def det(a, b):
#        return a[0] * b[1] - a[1] * b[0]
#
#    div = det(xdiff, ydiff)
#    if div == 0:
#       raise Exception('lines do not intersect')
#
#    d = (det(*line1), det(*line2))
#    x = det(d, xdiff) / div
#    y = det(d, ydiff) / div
#    return x, y
#
#print line_intersection((A, B), (C, D))


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
#        self.route.append(self._get_wp(initial_waypoint))
#        self.route_nodes.append(initial_waypoint)
        
        groute, gdist = self._create_greedy_plan(initial_waypoint)
        rroute, rdist = self._create_random_plan(initial_waypoint)
        
        print "Greedy: " + str(gdist) + " Random: " + str(rdist)
        
        if gdist < rdist:
            self.route = groute
        else:
            self.route = rroute
        
    
    def _create_greedy_plan(self, initial_waypoint):
        route=[]
        route.append(self._get_wp(initial_waypoint))
        
        local_targs = self.targets[:]
        
        while len(local_targs) > 0:
            cn = route[-1]
            min_dist =10000.0
            min_ind = -1
            for i in range(0, len(local_targs)):
                dist, ang = local_targs[i].coord - cn.coord
                if dist <= min_dist:
                   min_dist = dist
                   min_ind = i
            route.append(local_targs[min_ind])
            #self.route_nodes.append(local_targs[min_ind].name)
            local_targs.pop(min_ind)
        
        print [x.name for x in route]
        
        dist = 0
        for i in range(1, len(route)):
            d = route[i].coord - route[i-1].coord
            dist+=d[0]
        
        #print "TOTAL DIST: " + str(dist)
        return route, dist
            
            
        
    
    def _create_random_plan(self, initial_waypoint):
        route=[]
        route.append(self._get_wp(initial_waypoint))
        
        for i in self.targets:
            if i.name != initial_waypoint:
                route.append(i)
        
        dist = 0
        for i in range(1, len(route)):
            d = route[i].coord - route[i-1].coord
            dist+=d[0]
        
        #print "TOTAL DIST: " + str(dist)
        return route, dist