# from polygonhelper import *
from shapely.geometry import Polygon
from bbox import BBox

class World():
    # obstacles = [Obstacle1, Obstacle2]
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        for i in range(len(obstacles)):
            obstacles[i].id = i
        self.obstacles = obstacles

    def inRange(self, x, y):
        if 0 <= x <= self.width and 0 <= y <= self.height:
            return True
        return False

class Obstacle():
    #points is [(x1,y1), (x2,y2),...]
    def __init__(self, points, weight = 1):
        self.points = points
        self.polygon = Polygon(points)
        self.bbox = BBox(points)
        self.weight = weight
        self.id = None
    def __str__(self):
        return "id: {0}, weight: {1}".format(self.id, self.weight)