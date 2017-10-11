import urllib
import math
import numpy as np
import cv2

from map_coords import MapCoords

def url_to_image(url):
#download the image, convert it to a NumPy array, and then read it into OpenCV format
    resp = urllib.urlopen(url)
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    b_channel, g_channel, r_channel = cv2.split(image)
    alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255 #creating a dummy alpha channel image.
    image = cv2.merge((b_channel, g_channel, r_channel, alpha_channel))
    print "IMAGE SHAPE: ", image.shape
    return image


def get_res(size, zoom, lat):
    res =  156543.03 * math.cos(math.radians(lat))  #self.zoom2mpp[zoom]
    res = res / math.pow(2, zoom)
    return res


def getImageCluster(lat_deg, lon_deg, zoom, size):
    key="AIzaSyBRT6O0di_55wvbgYQGX8wwVKqHkrH-fnY"
    smurl = r"https://maps.googleapis.com/maps/api/staticmap?center={0},{1}&zoom={2}&size={4}x{4}&maptype=satellite&key={3}"
    imgurl=smurl.format(lat_deg, lon_deg, zoom, key, size)
    print "downloading %s" % (imgurl)
    image = url_to_image(imgurl)
    return image

class SatelliteImage(object):

    def __init__(self, lat_deg, lon_deg, zoom, size):
        self.size = size
        self.zoom = zoom
        self.res = get_res(self.size, self.zoom, lat_deg)
        
        self.limits=[]
        self.centre = MapCoords(lat_deg, lon_deg)       
        self.base_image = getImageCluster(self.centre.lat, self.centre.lon, self.zoom, self.size)
        

    def _coord2pix(self, point):
        dnorth = ((self.centre.northing- point.northing)/self.res) + (self.size/2)
        deast = ((point.easting - self.centre.easting)/self.res)  + (self.size/2)
        return deast, dnorth        
        
    def _latlong2pix(self, lat, lon):
        point = MapCoords(lat, lon)
        deast, dnorth = self._coord2pix(point)
        return deast, dnorth

    def draw_grid(self, grid, cell_size, colour, thickness=2):
        nx = len(grid)-1
        ny = len(grid[0])-1
        for i in range(0, len(grid[0])):
            mx0, my0 =self._coord2pix(grid[0][i]._get_rel_point(-cell_size/2,-cell_size/2))
            mx1, my1 =self._coord2pix(grid[nx][i]._get_rel_point(-cell_size/2,cell_size/2))
            cv2.line(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        mx0, my0 =self._coord2pix(grid[0][ny]._get_rel_point(cell_size/2,-cell_size/2))
        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
        cv2.line(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        for i in range(0, len(grid)):
            mx0, my0 =self._coord2pix(grid[i][0]._get_rel_point(-cell_size/2,-cell_size/2))
            mx1, my1 =self._coord2pix(grid[i][ny]._get_rel_point(cell_size/2,-cell_size/2))
            cv2.line(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)

        mx0, my0 =self._coord2pix(grid[nx][0]._get_rel_point(-cell_size/2,cell_size/2))
        mx1, my1 =self._coord2pix(grid[nx][ny]._get_rel_point(cell_size/2,cell_size/2))
        cv2.line(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        

    def draw_cell(self, cell, cell_size, colour, thickness=2):
        mx0, my0 =self._coord2pix(cell._get_rel_point(-cell_size/2,-cell_size/2))
        mx1, my1 =self._coord2pix(cell._get_rel_point(cell_size/2,cell_size/2))
        cv2.rectangle(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        

    def draw_coordinate(self, coord, colour, size=6, thickness=2):
        mx, my =self._coord2pix(coord)
        cv2.circle(self.base_image, (int(mx), int(my)), size, colour, thickness)

    def draw_list_of_coords(self, list_of_coords, colour, size=6, thickness=2):
        for i in list_of_coords:
            mx, my = self._coord2pix(i)
            cv2.circle(self.base_image, (int(mx), int(my)), size, colour, thickness)
    
    def draw_polygon(self, list_of_coords, colour, thickness=2):
        for i in range(1, len(list_of_coords)):
            mx0, my0 =self._coord2pix(list_of_coords[i-1])
            mx1, my1 =self._coord2pix(list_of_coords[i])
            cv2.line(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        mx0, my0 =self._coord2pix(list_of_coords[0])
        cv2.line(self.base_image, (int(mx0), int(my0)), (int(mx1), int(my1)), colour, thickness=thickness)
        