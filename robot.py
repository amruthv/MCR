from polygonhelper import *
from robotpolygon import *
import numpy as np
from heapq import * 
import math

class Robot():
    #polygonArr is a array of array of points (not polygon objects or point objects) e.g [[[1,2],[3,4]], [[5,5],[6,6]]] 
    def __init__(self, polygonArr):
        self.position = [Polygon(polygonPoints) for polygonPoints in polygonArr]

    def getLocation(self):
        return self.position[0].points[0].arr # first vertex of the first polygon of the robot

    # keep generating them until one is valid
    def generateRandomConfiguration(self):
        raise NotImplementedError()

    def distance(self, q1, q2):
        raise NotImplementedError()
