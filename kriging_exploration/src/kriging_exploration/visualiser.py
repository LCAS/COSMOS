#import numpy as np
import cv2
from kriging_exploration.satellite import SatelliteImage
from kriging_exploration.map_coords import MapCoords
from kriging_exploration.canvas import ViewerCanvas

class KrigingVisualiser(object):

    def __init__(self, lat_deg, lon_deg, zoom, size):
        print "Loading Satellite Image"
        self.satellite = SatelliteImage(lat_deg, lon_deg, zoom, size)
        self.base_image = self.satellite.base_image.copy()
        self.canvas = ViewerCanvas(self.base_image.shape, self.satellite.centre, self.satellite.res)

        #self.image = np.zeros(self.base_image.shape)
        print "canvas size"
        print self.canvas.image.shape
        self.refresh_image()
        print "init OK"        
        
        
    def refresh_image(self):
        self.image = cv2.addWeighted(self.canvas.image, 0.5, self.base_image, 0.9, 0)

        
    def draw_coordinate(self, lat, lon, colour='white', size=6, thickness=2):
        a = MapCoords(lat, lon)
        b = (255,255,255,100)
        self.canvas.draw_coordinate(a,b,size=size, thickness=thickness)
        self.refresh_image()


#    def _draw_coordinate(self, coord, colour, size=6, thickness=2):
#        mx, my =self._coord2pix(coord)
#        cv2.circle(self.base_image, (int(mx), int(my)), size, colour, thickness)