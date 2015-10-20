from mcrhelper import MCRHelper
from shapely.geometry import Polygon
from bbox import BBox
import numpy as np
import random

class SimpleMCRHelper(MCRHelper):
    def __init__(self, robot, world, goal):
        self.robot = robot
        self.world = world
        self.goal = goal
        self.useBBoxChecker = True

    def collisionsAtQ(self, q):
        if self.useBBoxChecker:
            return self.coverWithBBox(q)
        else:
            return self.naiiveCover(q)

    def coverWithBBox(self, q):
        self.robot.moveToConfiguration(q)
        collisions = set()
        for obstacle in self.world.obstacles:
            allRobotPoints = [pt for polyPoints in self.robot.position for pt in polyPoints]
            allRobotBBox = BBox(allRobotPoints)
            collisionFree = True
            for obstacle in self.world.obstacles:
                if allRobotBBox.intersectsBBox(obstacle.bbox):
                    collisionFree = False
                    break
            if collisionFree:
                return collisions

            #else check each robot polygon bbox
            for polygonPts in self.robot.position:
                bboxRobotPoly = BBox(polygonPts)
                polygon = None
                for obstacle in self.world.obstacles:
                    if bboxRobotPoly.intersectsBBox(obstacle.bbox):
                        polygon = Polygon(polygonPts)
                        if polygon.intersects(obstacle.polygon):
                            collisions.add(obstacle)
        return collisions

    def naiiveCover(self, q):
        self.robot.moveToConfiguration(q)
        collisions = set()
        for polygon in self.robot.position:
            poly = Polygon(polygon)
            for obstacle in self.world.obstacles:
                if poly.intersects(obstacle.polygon):
                    collisions.add(obstacle)
        return collisions

    def sampleConfig(self, goal):
        if random.random() < 0.1:
            return goal
        return self.robot.generateRandomConfiguration()

    def generateInBetweenConfigs(self, q_from, q_to):
        stepsToCheck = 10
        configurations = []
        q_from_np = np.array(q_from)
        q_to_np = np.array(q_to)
        q_delta = q_to_np - q_from_np
        for i in range(1, stepsToCheck):
            q = q_from_np + q_delta * float(i) / stepsToCheck
            configurations.append(q)
        return configurations

    def distance(self, q1, q2):
        return self.robot.distance(q1, q2)
