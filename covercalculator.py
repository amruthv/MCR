import polygonhelper
import numpy as np
from cover import Cover

class CoverCalculator():
    def __init__(self, robot, world):
        self.robot = robot
        self.stepsToCheck = 10
        self.world = world

    #check that they are in bounds elsewhere, here we assume they are
    def edgeCover(self, q_from, q_to):
        #make empty cover and build it up
        edge_cover = Cover(set())
        configurationsToCheck = self.buildInBetweenConfigurations(q_from, q_to)
        for q in configurationsToCheck:
            coverQ = self.cover(q)
            edge_cover = edge_cover.mergeWith(coverQ)         
        return edge_cover

    def cover(self, q):
        coverQ = set()
        self.robot.moveToConfiguration(q)
        for polygon in self.robot.position:
            for obstacle in self.world.obstacles:
                if polygon.intersects(obstacle.polygon):
                    coverQ.add(obstacle)
        return Cover(coverQ)

    def buildInBetweenConfigurations(self, q_from, q_to):
        # print 'q_from', q_from
        # print 'q_to', q_to
        configurations = []
        q_from_np = np.array(q_from)
        q_to_np = np.array(q_to)
        q_delta = q_to_np - q_from_np
        for i in range(1, self.stepsToCheck + 1):
            q = q_from_np + q_delta * float(i) / self.stepsToCheck
            configurations.append(q)
        return configurations




    