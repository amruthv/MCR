# from polygonhelper import *
from robotpolygon import *
import numpy as np
from heapq import * 
import math
import random

class PolygonalRobot():
    #polygonArr is a array of array of points (not polygon objects or point objects) e.g [[[1,2],[3,4]], [[5,5],[6,6]]] 
    def __init__(self, polygonArr, world, goal, rotationDegree = 0):
        self.position = [Polygon(polygonPoints) for polygonPoints in polygonArr]
        self.goal = goal
        self.world = world
        self.rotationDegree = rotationDegree

    def generateRandomConfiguration(self, goalBias):
        if random.random() < goalBias:
            q_rand = self.goal
        else:
            x = random.randint(0, self.world.width)
            y = random.randint(0, self.world.height)
            theta = random.randint(-180, 180)
            q_rand = [x, y, theta]
        return q_rand

    def moveToConfiguration(self, q):
        x,y,theta = q
        deltaTheta = (theta - self.rotationDegree) % 360
        self.rotate(deltaTheta)
        self.moveAbsolute(x,y)

    def getLocation(self):
        return self.position[0].points[0].arr # first vertex of the first polygon of the robot

    def moveRelative(self, dx, dy):
        self.position = [polygon.translate(dx,dy) for polygon in self.position]

    def moveAbsolute(self, x, y):
        # first figure out the diff from where the robot currently is so we can figure out the translation needed
        dx = x - self.getLocation()[0]
        dy = y - self.getLocation()[1]
        self.moveRelative(dx,dy)

    # mutates current robot
    def rotate(self, degrees):
        rotatedPolygons = []
        referencePoint = self.position[0].points[0]
        for polygon in self.position:
            rotatedPolygon = polygon.rotate(degrees, referencePoint)
            rotatedPolygons.append(rotatedPolygon)
        self.position = rotatedPolygons
        self.rotationDegree = (self.rotationDegree + degrees) % 360

    def inBounds(self):
        for polygon in self.position:
            for point in polygon.points:
                if not self.world.inRange(point.x, point.y):
                    return False
        return True
