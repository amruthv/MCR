# from polygonhelper import *
from robot import *
import numpy as np
import math
import random
from shapely.geometry import Polygon

class MovableLinkRobot(Robot):
    #polygonArr is an array of tuples (angle, array of points -- not polygon objects or point objects) e.g [(0,[([1,2],[3,4]]), (20,[[5,5],[6,6]]]) 
    def __init__(self, polygonInfoArray, world):
        self.polygons = self.movePolygonsToOrigin(polygonInfoArray)
        self.numJoints = len(polygonInfoArray)
        referenceTransforms, inverseReferenceTransforms = self.makeInitialTransforms(polygonInfoArray)
        self.referenceTransforms = referenceTransforms
        self.inverseReferenceTransforms = inverseReferenceTransforms
        # assume the robot starts off flat, but we can extract the x,y coordinate of the first leg from the provided points
        firstCoordinate = list(polygonInfoArray[0][1][0])   
        self.initialOrigin = firstCoordinate
        self.moveToConfiguration(firstCoordinate + [0] * self.numJoints)
        self.world = world

    def movePolygonsToOrigin(self, polygonInfoArray):
        polygons = []
        for (_, polygonPoints) in polygonInfoArray:
            pointsMovedToOrigin = []
            deltaX, deltaY = polygonPoints[0]
            for (x,y) in polygonPoints:
                pointsMovedToOrigin.append((x - deltaX, y - deltaY))
            polygons.append(pointsMovedToOrigin)
        return polygons

    def makeInitialTransforms(self, polygonInfoArray):
        referenceTransforms = []
        inverseReferenceTransforms = []
        for polygonAngle, polygonPoints in polygonInfoArray:
            polygonRotationPoint = polygonPoints[0]
            radians = math.radians(polygonAngle)
            cosTheta = math.cos(radians)
            sinTheta = math.sin(radians)
            transform = np.array([[cosTheta, -sinTheta, polygonRotationPoint[0]], 
                [sinTheta, cosTheta, polygonRotationPoint[1]],
                [0, 0, 1]])
            referenceTransforms.append(transform)
            inverseReferenceTransforms.append(np.linalg.inv(transform))
        return referenceTransforms, inverseReferenceTransforms

    def findTransformsForConfiguration(self, configuration):
        transforms = []
        for jointNum, jointAngle in enumerate(configuration):
            radians = math.radians(jointAngle)
            cosTheta = math.cos(radians)
            sinTheta = math.sin(radians)
            rotationMatrix = np.array([[cosTheta, -sinTheta, 0],
                                [sinTheta, cosTheta, 0],
                                [0, 0, 1]])
            if jointNum == 0:
                transform = self.referenceTransforms[jointNum].dot(rotationMatrix)
            else:
                previousJointTransform = transforms[jointNum - 1]
                transformToPreviousFrame = self.inverseReferenceTransforms[jointNum - 1].dot(self.referenceTransforms[jointNum])
                transform = previousJointTransform.dot(transformToPreviousFrame).dot(rotationMatrix)
            transforms.append(transform)
        return transforms


    # [x,y, joint angles]
    def moveToConfiguration(self, q):
        transforms = self.findTransformsForConfiguration(q[2:])
        positionOfRobot = []
        for polygonNumber, polygonPosition in enumerate(self.polygons):
            transformedPolygonPoints = []
            for xPoint, yPoint in polygonPosition:
                transformedPoint = transforms[polygonNumber].dot(np.array([xPoint, yPoint, 1]))
                # transformedPolygonPoints.append(Point(transformedPoint[0], transformedPoint[1]))
                transformedPolygonPoints.append((transformedPoint[0] + q[0] - self.initialOrigin[0], transformedPoint[1] + q[1] - self.initialOrigin[1]))
            positionOfRobot.append(Polygon(transformedPolygonPoints))
        self.position = positionOfRobot

    # keep generating them until one is valid
    def generateRandomConfiguration(self):
        # [x, y, joint_angles]
        originalPosition = self.position
        while True:
            q_rand = [random.uniform(0,self.world.width - 1), random.uniform(0, self.world.height -1)] + [random.uniform(-180,180) for i in range(self.numJoints)]
            self.moveToConfiguration(q_rand)
            if self.inBounds():
                # reset position
                self.position = originalPosition
                return tuple(q_rand)

    def makeRotationMatrix(self, rotationAngle):
        radians = math.radians(rotationAngle)
        cosTheta = math.cos(radians)
        sinTheta = math.sin(radians)
        return np.array([[cosTheta, -sinTheta, 0], [sinTheta, cosTheta, 0], [0, 0, 1]])

    def inBounds(self):
        for polygon in self.position:
            for point in polygon.exterior.coords[:-1]:
                if not self.world.inRange(point[0], point[1]):
                    return False
        return True

    def distance(self, q1, q2):
        assert(len(q1) == len(q2))
        angleFactor = 0.2
        # get euclidean distance for the origin of the two configurations
        euclideanDistance = math.sqrt((q1[0] - q2[0]) ** 2 + (q1[1] - q2[1]) ** 2)
        # get angle distance for the joint angles 
        jointAngleSum = 0
        for i in range(2, len(q1)):
            jointAngleSum += (q1[i] - q2[i]) ** 2
        jointAngleDistance = math.sqrt(jointAngleSum)
        distance = euclideanDistance + jointAngleDistance * angleFactor
        return distance

