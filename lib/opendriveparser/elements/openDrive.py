
class OpenDrive(object):

    def __init__(self):
        self._header = None
        self._roads = {}
        self._junctions = {}
        self._junctionGroups = []

    @property
    def header(self):
        return self._header

    @header.setter
    def header(self, value):
        if not isinstance(value, Header):
            raise TypeError("Value must be instance of Header.")

        self._header = value

    @property
    def roads(self):
        return self._roads

    def getRoad(self, id):
        for road in self._roads:
            if road.id == id:
                return road

        return None


    @property
    def junctions(self):
        return self._junctions

    @property
    def junctionGroups(self):
        return self._junctionGroups



class Header(object):

    def __init__(self):
        self._revMajor = None
        self._revMinor = None
        self._name = None
        self._version = None
        self._date = None
        self._north = None
        self._south = None
        self._east = None
        self._west = None
        self._vendor = None
        self._lat = None
        self._lon = None

    @property
    def lat(self):
        return self._lat

    @lat.setter
    def lat(self, value):
        self._lat = str(value)

    @property
    def lon(self):
        return self._lon

    @lon.setter
    def lon(self, value):
        self._lon = str(value)
