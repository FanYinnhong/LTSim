import os
from lxml import etree
from io import StringIO, BytesIO
_ME_PATH = os.path.abspath(os.path.dirname(__file__))
DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../../data/xml'))

root = etree.Element("Catalog")
country = etree.SubElement(root,'country', {'name':'Liechtenstein'})
rank = etree.SubElement(country,'rank')
rank.text = '1'

m1 = etree.Element("mobile")
root.append(m1)
b1 = etree.SubElement(m1, "brand")
b1.text = "Redmi"
b2 = etree.SubElement(m1, "price")
b2.text="6999"

tree = etree.ElementTree(root)
filename = "Catalog.xml"
filepath = os.path.join(DATA_PATH, filename)

with open(filepath, "w") as f:
    f.write(etree.tostring(tree, pretty_print=True, encoding="unicode"))