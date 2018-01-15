#import urllib
#import math
import numpy as np
import cv2
from matplotlib import colors as mcolors

from map_coords import MapCoords


class ViewerCanvas(object):

    def __init__(self, shape, centre, res):
        self.size = shape[0]
        self.res = res
        self.centre = centre
        self.image = np.zeros(shape, dtype=np.uint8)
        self.shape = shape

        
    def _coord2pix(self, point):
        dnorth = ((self.centre.northing- point.northing)/self.res) + (self.size/2)
        deast = ((point.easting - self.centre.easting)/self.res)  + (self.size/2)
        return deast, dnorth        
        
    def _pix2coord(self, x, y):
        xcord = (x - (self.size/2))*self.res
        ycord = -(y - (self.size/2))*self.res
        click_coord = self.centre._get_rel_point(xcord,ycord)
        return click_coord
        
    def _latlong2pix(self, lat, lon):
        point = MapCoords(lat, lon)
        deast, dnorth = self._coord2pix(point)
        return deast, dnorth

    def clear_image(self):
        self.image = np.zeros(self.shape, dtype=np.uint8)


    def draw_coordinate(self, coord, colour, size=6, thickness=2, alpha=128):
        mx, my =self._coord2pix(coord)        
        b = [255*x for x in mcolors.hex2color(mcolors.cnames[colour])]
        b = b[::-1]
        b.append(alpha)
        cv2.circle(self.image, (int(mx), int(my)), size, b, thickness)


    def draw_waypoints(self, grid, colour, thickness=2):
        for i in range(0, len(grid)):
            for j in range(0, len(grid[0])):
                mx0, my0 = self._coord2pix(grid[i][j])
                cv2.circle(self.image, (int(mx0), int(my0)), 2, colour, thickness=thickness)
        
#        for i in range(0, len(grid[0])):
#            mx0, my0 =self._coord2pix(grid[0][i]._get_rel_point(-cell_size/2,-cell_size/2))
#            mx1, my1 =self._coord2pix(grid[nx][i]._get_rel_point(-cell_size/2,cell_size/2))
#            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
#
#        mx0, my0 =self._coord2pix(grid[0][ny]._get_rel_point(cell_size/2,-cell_size/2))
#        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
#        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
#
#        for i in range(0, len(grid)):
#            mx0, my0 =self._coord2pix(grid[i][0]._get_rel_point(-cell_size/2,-cell_size/2))
#            mx1, my1 =self._coord2pix(grid[i][ny]._get_rel_point(cell_size/2,-cell_size/2))
#            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
#
#        mx0, my0 =self._coord2pix(grid[nx][0]._get_rel_point(-cell_size/2,cell_size/2))
#        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
#        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        
    def draw_grid(self, grid, cell_size, colour, thickness=2):
        nx = len(grid)-1
        ny = len(grid[0])-1
        for i in range(0, len(grid[0])):
            mx0, my0 =self._coord2pix(grid[0][i]._get_rel_point(-cell_size/2,-cell_size/2))
            mx1, my1 =self._coord2pix(grid[nx][i]._get_rel_point(-cell_size/2,cell_size/2))
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        mx0, my0 =self._coord2pix(grid[0][ny]._get_rel_point(cell_size/2,-cell_size/2))
        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        for i in range(0, len(grid)):
            mx0, my0 =self._coord2pix(grid[i][0]._get_rel_point(-cell_size/2,-cell_size/2))
            mx1, my1 =self._coord2pix(grid[i][ny]._get_rel_point(cell_size/2,-cell_size/2))
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        mx0, my0 =self._coord2pix(grid[nx][0]._get_rel_point(-cell_size/2,cell_size/2))
        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        

    def draw_cell(self, cell, cell_size, colour, thickness=2):
        mx0, my0 =self._coord2pix(cell._get_rel_point(-cell_size/2,-cell_size/2))
        mx1, my1 =self._coord2pix(cell._get_rel_point(cell_size/2,cell_size/2))
        cv2.rectangle(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        

    def draw_list_of_coords(self, list_of_coords, colour, size=6, thickness=2):
        for i in list_of_coords:
            mx, my = self._coord2pix(i)
            cv2.circle(self.image, (int(mx), int(my)), size, colour, thickness)
    
    def draw_polygon(self, list_of_coords, colour, thickness=2):
        for i in range(1, len(list_of_coords)):
            mx0, my0 =self._coord2pix(list_of_coords[i-1])
            mx1, my1 =self._coord2pix(list_of_coords[i])
            cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        mx0, my0 =self._coord2pix(list_of_coords[0])
        cv2.line(self.image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
    
    def put_text(self,text):
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.image, text, (int(520), int(20)), font, 0.8, (200, 200, 200), 2)
        
        
        
        