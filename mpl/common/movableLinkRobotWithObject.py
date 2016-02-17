import numpy as np
import math
import random
import pdb
from shapely.geometry import Polygon

from world import SelfObstacle
from bbox import BBox
from robot import *
from configuration import Configuration

class MovableLinkRobotWithObject(Robot):
    #polygonArr is an array of tuples (angle, array of points -- not polygon objects or point objects) e.g [(0,[([1,2],[3,4]]), (20,[[5,5],[6,6]]]) 
    def __init__(self, robotPolygonInfoArray, objectPolygonInfoArray, world):
        self.robotPolygons = self.movePolygonsToOrigin([polygon for _, polygon in robotPolygonInfoArray])
        self.heldObjectPolygons = self.movePolygonsToOrigin(objectPolygonInfoArray)
        self.numJoints = len(robotPolygonInfoArray)
        referenceTransforms, inverseReferenceTransforms = self.makeInitialTransforms(robotPolygonInfoArray)
        self.referenceTransforms = referenceTransforms
        self.inverseReferenceTransforms = inverseReferenceTransforms
        referenceObjectTransforms, _ = self.makeInitialTransforms([(0, objectPolygon) for objectPolygon in objectPolygonInfoArray])
        self.referenceObjectTransforms = referenceObjectTransforms

        # we can extract the x,y coordinate of the first leg from the provided points
        firstCoordinate = list(robotPolygonInfoArray[0][1][0])  
        self.heldObstacleOffsets = [[objectPolygon[0][0] - robotPolygonInfoArray[-1][1][0][0], \
                                objectPolygon[0][1] - robotPolygonInfoArray[-1][1][0][1]] for \
                                objectPolygon in objectPolygonInfoArray]
        self.initialOrigin = firstCoordinate
        # self.moveToConfiguration(firstCoordinate + [0] * self.numJoints)
        self.world = world

    def movePolygonsToOrigin(self, polygonInfoArray):
        polygons = []
        for polygonPoints in polygonInfoArray:
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
            cosTheta = math.cos(polygonAngle)
            sinTheta = math.sin(polygonAngle)
            transform = np.array([[cosTheta, -sinTheta, polygonRotationPoint[0]], 
                [sinTheta, cosTheta, polygonRotationPoint[1]],
                [0, 0, 1]])
            referenceTransforms.append(transform)
            inverseReferenceTransforms.append(np.linalg.inv(transform))
        return referenceTransforms, inverseReferenceTransforms

    def findTransformsForConfiguration(self, jointAngles):
        # add on 0s for the held object polygons
        # configuration += [0] * len(self.heldObjectPolygons)
        transforms = []
        for jointNum, jointAngle in enumerate(jointAngles):
            cosTheta = math.cos(jointAngle)
            sinTheta = math.sin(jointAngle)
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

        lastJointTransform = transforms[-1]
        #treat theta as 0 as it is 0 rotation from the last joint
        objectRotationMatrix = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        for i in range(len(self.heldObjectPolygons)):
            lastJointTransformInverse = self.inverseReferenceTransforms[-1].dot(self.referenceObjectTransforms[i])
            transforms.append(lastJointTransform.dot(lastJointTransformInverse).dot(objectRotationMatrix))
        return transforms


    # [x,y, joint angles]
    def moveToConfiguration(self, q):
        cartesianParameters = q.cartesianParameters
        transforms = self.findTransformsForConfiguration(q.angleParameters)
        transformsForRobot = transforms[:len(self.robotPolygons)]
        transformsForObjectPolygons = transforms[len(self.robotPolygons):]
        positionOfRobot = []
        for polygonNumber, polygonPosition in enumerate(self.robotPolygons):
            transformedPolygonPoints = []
            for xPoint, yPoint in polygonPosition:
                transformedPoint = transformsForRobot[polygonNumber].dot(np.array([xPoint, yPoint, 1]))
                transformedPolygonPoints.append((transformedPoint[0] + cartesianParameters[0] - self.initialOrigin[0], transformedPoint[1] + cartesianParameters[1] - self.initialOrigin[1]))
            positionOfRobot.append(transformedPolygonPoints)
        self.robotPosition = positionOfRobot

        positionOfHeldObject = []
        for objectPolygonNum, objectPolygon in enumerate(self.heldObjectPolygons):
            transformedObjectPolygonPoints = []
            for xPoint, yPoint in objectPolygon:
                transformedPoint = transformsForObjectPolygons[objectPolygonNum].dot(np.array([xPoint, yPoint, 1]))
                transformedObjectPolygonPoints.append((transformedPoint[0] + cartesianParameters[0] - self.initialOrigin[0],
                                                       transformedPoint[1] + cartesianParameters[1] - self.initialOrigin[1]))
            positionOfHeldObject.append(transformedObjectPolygonPoints)
        self.heldObjectPosition = positionOfHeldObject

    # keep generating them until one is valid
    def generateRandomConfiguration(self):
        # [x, y, joint_angles]
        while True:
            q_rand = Configuration.getRandomConfiguration(0, self.world.width - 1, 0, self.world.height - 1, self.numJoints)
            if self.doesHeldObjectOverlapRobotAtQ(q_rand):
                continue
            self.moveToConfiguration(q_rand)
            if self.inBounds():
                return q_rand

    def makeRotationMatrix(self, rotationAngle):
        cosTheta = math.cos(rotationAngle)
        sinTheta = math.sin(rotationAngle)
        return np.array([[cosTheta, -sinTheta, 0], [sinTheta, cosTheta, 0], [0, 0, 1]])

    def inBounds(self):
        allPolygons = self.robotPosition + self.heldObjectPosition
        for polygon in allPolygons:
            for point in polygon:
                if not self.world.inRange(point[0], point[1]):
                    return False
        return True

    def doesHeldObjectOverlapRobotAtQ(self, q):
        self.moveToConfiguration(q)
        robotPolygonObjects = [Polygon(robotPolygon) for robotPolygon in self.robotPosition][:-1]
        heldObjectPolygonObjects = [Polygon(heldObjectPolygon) for heldObjectPolygon in self.heldObjectPosition]
        for heldObjectPolygonObject in heldObjectPolygonObjects:
            if any(heldObjectPolygonObject.overlaps(robotPolygonObject) for robotPolygonObject in robotPolygonObjects):
                return True
        return False

    def collisionsAtQ(self, q):
        self.moveToConfiguration(q)
        #make sure that held object doesn't collide with robot
        if self.doesHeldObjectOverlapRobotAtQ(q):
            return set([SelfObstacle()])
        
        allPolygons = self.robotPosition + self.heldObjectPosition
        allPoints = [pt for polyPoints in allPolygons for pt in polyPoints]
        allRobotBBox = BBox(allPoints)
        allPolygonObjects = [Polygon(polygon) for polygon in allPolygons]
        collisionFree = True    
        collisions = set()
        for obstacle in self.world.obstacles:
            if allRobotBBox.intersectsBBox(obstacle.bbox):
                collisionFree = False
                break
        if collisionFree:
            return collisions
        # else check each robot polygon bbox
        for polygonNum, polygonPts in enumerate(allPolygons):
            bboxRobotPoly = BBox(polygonPts)
            polygon = None
            for obstacle in self.world.obstacles:
                if bboxRobotPoly.intersectsBBox(obstacle.bbox):
                    polygonObj = allPolygonObjects[polygonNum]
                    if polygonObj.intersects(obstacle.polygon):
                        collisions.add(obstacle)
        return collisions

    def distance(self, q1, q2):
        # return self.angleDiscount(q1, q2)
        # return self.forwardKinDistance(q1, q2)
        return self.l2Norm(q1.cartesianParameters + q1.angleParameters, q2.cartesianParameters + q2.angleParameters)

    def forwardKinDistance(self, q1, q2):
        self.moveToConfiguration(q1)
        q1Position = self.robotPosition
        self.moveToConfiguration(q2)
        q2Position = self.robotPosition
        distance = 0.0
        for i in range(len(q1Position)):
            for j in range(len(q1Position[i])):
                distance += self.l2Norm(q1Position[i][j], q2Position[i][j])
        return distance

    def l2Norm(self, q1, q2):
        return np.linalg.norm(np.array(q2) - np.array(q1))

    def angleDiscount(self, q1, q2):
        angleFactor = 0.2
        # get euclidean distance for the origin of the two configurations
        cartesianQ1 = q1.cartesianParameters
        cartesianQ2 = q2.cartesianParameters
        angleQ1 = q1.angleParameters
        angleQ2 = q2.angleParameters
        euclideanDistance = math.sqrt((cartesianQ1[0] - cartesianQ2[0]) ** 2 + (cartesianQ1[1] - cartiesianQ2[1]) ** 2)
        # get angle distance for the joint angles 
        jointAngleSum = 0
        for i in range(len(angleQ1)):
            jointAngleSum += (angleQ1[i] - angleQ2[i]) ** 2
        jointAngleDistance = math.sqrt(jointAngleSum)
        distance = euclideanDistance + jointAngleDistance * angleFactor
        return distance

