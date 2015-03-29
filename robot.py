from polygonhelper import *
from robotpolygon import *
import numpy as np
from heapq import * 
import math

#point is (x,y,theta) and boundary points is list of (x,y)
def isPointInWorldBoundaries(point, boundaryPoints):
    minX = min(corner[0] for corner in boundaryPoints)
    maxX = max(corner[0] for corner in boundaryPoints)
    minY = min(corner[1] for corner in boundaryPoints)
    maxY = max(corner[1] for corner in boundaryPoints)
    return minX <= point[0] <= maxX and minY <= point[1] <= maxY

class Robot():
    #polygonArr is a array of array of points (not polygon objects or point objects) e.g [[[1,2],[3,4]], [[5,5],[6,6]]] 
    def __init__(self, polygonArr, world, rotationDegree = 0):
        self.position = [Polygon(polygonPoints) for polygonPoints in polygonArr]
        self.world = world
        self.rotationDegree = rotationDegree

    def getLocation(self):
        return self.position[0].points[0].arr # first vertex of the first polygon of the robot

    def moveRelative(self, dx, dy):
        self.position =[polygon.translate(dx,dy) for polygon in self.position]

    def moveAbsolute(self, x, y):
        # first figure out the diff from where the robot currently is so we can figure out the translation needed
        dx = x - self.getLocation()[0]
        dy = y - self.getLocation()[1]
        self.moveRelative(dx,dy)

    def moveInWorldBounds(self,dx,dy):
        for polygon in self.position:
            shiftedPolygon = polygon.translate(dx,dy)
            for point in shiftedPolygon.points:
                if not self.world.inRange(point.x,point.y):
                    return False
        return True

    def rotate(self, degrees):
        rotatedPolygons = []
        referencePoint = self.position[0].points[0]
        for polygon in self.position:
            rotatedPolygon = polygon.rotate(degrees, referencePoint)
            rotatedPolygons.append(rotatedPolygon.points)
        return Robot(rotatedPolygons, self.world, self.rotationDegree + degrees)
