import os
from lxml import etree
from io import StringIO, BytesIO
from lib.opendriveparser import parse_opendrive
import numpy as np

_ME_PATH = os.path.abspath(os.path.dirname(__file__))
DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../data/xml'))
filename = "Town01.xodr"
filepath = os.path.join(DATA_PATH, filename)
with open(filepath, 'r') as f:
    parser = etree.XMLParser()
    rootNode = etree.parse(f, parser).getroot()
    roadNetwork = parse_opendrive(rootNode)
