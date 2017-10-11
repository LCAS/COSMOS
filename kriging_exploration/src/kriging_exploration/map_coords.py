import utm

class MapCoords(object):

    def __init__(self, lat, lon):
        self.lat =lat
        self.lon =lon
        a = utm.from_latlon(lat, lon)
        self.northing = a[1]
        self.easting = a[0]
        self.zone_number = a[2]
        self.zone_letter = a[3]

    def _get_rel_point(self, easting, northing):
        a = utm.to_latlon(self.easting + easting, self.northing + northing, self.zone_number, self.zone_letter)
        return MapCoords(a[0], a[1])


    def __repr__(self):
        a = dir(self)
        b = []
        s = ''
        for i in a:
            if not i.startswith('_'):
                b.append(str(i))
        for i in b:
                s = s + str(i) + ': ' + str(self.__getattribute__(i)) + '\n'
        return s