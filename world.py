from polygonhelper import *

class World():
    # obstacles = list of polygons
    def __init__(self, width, height):
        self.width = width
        self.height = height

    def inRange(self, x, y):
        if 0 <= x <= self.width and 0 <= y <= self.height:
            return True
        return False