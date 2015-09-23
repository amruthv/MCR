# from polygonhelper import *
from shapely.geometry import Polygon
from bbox import BBox

class World():
    # obstacles = [Obstacle1, Obstacle2]
    def __init__(self, width, height, obstacles, assignIds = True):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        if assignIds:
            self.assignIds()

    def assignIds(self):
        for i in range(len(self.obstacles)):
            self.obstacles[i].setId(i)

    def inRange(self, x, y):
        if 0 <= x <= self.width and 0 <= y <= self.height:
            return True
        return False

class Obstacle():
    #points is [(x1,y1), (x2,y2),...]

    def getId(self):
        raise NotImplementedError()

    def getWeight(self):
        raise NotImplementedError()

    def setId(self, obstacleId):
        raise NotImplementedError()

    def __hash__(self):
        return self.getId()

class SimpleObstacle(Obstacle):
    def __init__(self, points, weight = 1):
        self.points = points
        self.polygon = Polygon(points)
        self.bbox = BBox(points)
        self.weight = weight
        self.id = None

    def getId(self):
        return self.id

    def setId(self, obstacleId):
        self.id = obstacleId

    def getWeight(self):
        return self.weight

    def setWeight(self, w):
        self.weight = w

    def __str__(self):
        return "id: {0}, weight: {1}".format(self.id, self.weight)