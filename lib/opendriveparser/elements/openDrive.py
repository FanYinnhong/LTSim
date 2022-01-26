class OpenDrive(object):

    def __init__(self):
        self._header = None
        self._roads = []
        self._junctions = []
        self._junctionGroups = []

    @property
    def header(self):
        return self._header

    @property
    def roads(self):
        return self._roads

    @property
    def junctions(self):
        return self._junctions

    @property
    def junctioinGroups(self):
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
