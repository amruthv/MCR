from shapely.geometry import Polygon
from bbox import BBox

class World():
    # obstacles = [Obstacle1, Obstacle2]
    def __init__(self, width, height, obstacles, assignIds = True):
        self.width = width
        self.height = height
        self.obstacles = obstacles

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
        return self.getName().__hash__()

class SimpleObstacle(Obstacle):
    def __init__(self, name, points, weight = 1):
        self.points = points
        self.polygon = Polygon(points)
        self.bbox = BBox(points)
        self.weight = weight
        self.name = name

    def getName(self):
        return self.name

    def getWeight(self):
        return self.weight

    def setWeight(self, w):
        self.weight = w

    def __str__(self):
        return "({0}, weight: {1})".format(self.name, self.weight)
    def __repr__(self):
        return self.__str__()

class SelfObstacle(Obstacle):
    def __init__(self):
        pass    
    def getName(self):
        return 'self'

    def getWeight(self):
        return float('inf')
