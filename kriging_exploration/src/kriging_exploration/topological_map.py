import utm
import yaml

import numpy as np

from map_coords import MapCoords


def coord_in_poly(point, limits):
    x=point.easting
    y=point.northing
    
    n = len(limits)
    inside = False
#
    p1x = limits[0].easting
    p1y = limits[0].northing
    
    for i in range(n+1):
        p2x = limits[i % n].easting
        p2y = limits[i % n].northing
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y
    return inside


class TopoNode(object):
    def __init__(self, name, coord, ind):
        self.name = name
        self.coord = coord  #Node coordinates
        self.ind = ind      #Grid Indexes



class TopoMap(object):
    def __init__(self, grid):
        self.waypoints=[]
        self.calculate_waypoints(grid)
        
    def calculate_waypoints(self, grid):
        ind = 0
        for i in range(0, len(grid.cells)):
            for j in range(0, len(grid.cells[0])):
                if coord_in_poly(grid.cells[i][j], grid.limits):
                    name = 'Waypoint%003d' %ind
                    d = TopoNode(name, grid.cells[i][j], (i, j))
                    self.waypoints.append(d)
                    ind+=1
                    print name
        #print len(self.waypoints)
