import numpy as np
from cover import Cover
from shapely.geometry import Polygon
from bbox import BBox

class CoverCalculator():
    def __init__(self, robot, world, useBoundingBoxes = True):
        self.robot = robot
        self.stepsToCheck = 10
        self.world = world
        self.useBB = True

    #check that the configurations are in bounds elsewhere, here we assume they are. only checks configurations between the two not the endpoints
    def edgeCover(self, q_from, q_to):
        #make empty cover and build it up
        edge_cover = Cover(set())
        configurationsToCheck = self.buildInBetweenConfigurations(q_from, q_to)
        for q in configurationsToCheck:
            coverQ = self.cover(q)
            edge_cover = edge_cover.mergeWith(coverQ)         
        return edge_cover

    def cover(self, q):
        if self.useBB:
            return self.coverWithBB(q)
        else:
            return self.naiiveCover(q)

    def coverWithBB(self, q):
        coverQ = set()
        self.robot.moveToConfiguration(q)
        allRobotPoints = [pt for polyPoints in self.robot.position for pt in polyPoints]
        allRobotBBox = BBox(allRobotPoints)
        collisionFree = True
        for obstacle in self.world.obstacles:
            if allRobotBBox.intersectsBBox(obstacle.bbox):
                collisionFree = False
                break
        if collisionFree:
            return Cover(coverQ)

        #else check each robot polygon bbox
        for polygonPts in self.robot.position:
            bboxRobotPoly = BBox(polygonPts)
            polygon = None
            for obstacle in self.world.obstacles:
                if bboxRobotPoly.intersectsBBox(obstacle.bbox):
                    polygon = Polygon(polygonPts)
                    if polygon.intersects(obstacle.polygon):
                        coverQ.add(obstacle)
        return Cover(coverQ)

    def naiiveCover(self, q):
        coverQ = set()
        self.robot.moveToConfiguration(q)
        for polygon in self.robot.position:
            poly = Polygon(polygon)
            for obstacle in self.world.obstacles:
                if poly.intersects(obstacle.polygon):
                    coverQ.add(obstacle)
        return Cover(coverQ)

    def buildInBetweenConfigurations(self, q_from, q_to):
        # print 'q_from', q_from
        # print 'q_to', q_to
        configurations = []
        q_from_np = np.array(q_from)
        q_to_np = np.array(q_to)
        q_delta = q_to_np - q_from_np
        for i in range(1, self.stepsToCheck):
            q = q_from_np + q_delta * float(i) / self.stepsToCheck
            configurations.append(q)
        return configurations




    