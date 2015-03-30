import polygonhelper
import numpy as np

class CoverCalculator():
    def __init__(self, robot, world):
        self.robot = robot
        self.stepsToCheck = 10
        self.world = world

    #check that they are in bounds elsewhere, here we assume they are
    def edgeCover(self, q_from, q_to):
        edgeCover = set()
        configurationsToCheck = self.buildInBetweenConfigurations(q_from, q_to)
        for q in configurationsToCheck:
            edgeCover = edgeCover.union(cover(q))
                        
        return edgeCover

    def cover(self, q):
        cover = set()
        self.robot.moveToConfiguration(q)
        for polygon in self.robot.position:
            for obstacle in self.world.obstacles:
                if polygon.doesIntersect(obstacle.polygon):
                    cover.add(obstacle.id)
        return cover

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




    