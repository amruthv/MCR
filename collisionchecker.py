import polygonhelper
import numpy as np

class CollisionChecker():
    def __init__(self, robot, obstacleList, isParamDegreeList):
        self.robot = robot
        self.stepsToCheck = 10
        self.obstacles = obstacleList
        self.isParamDegreeList = isParamDegreeList
    def validateTrajectory(self, q_from, q_to):
        configurationsToCheck = self.buildInBetweenConfigurations(q_from, q_to)
        for q in configurationsToCheck:
            self.robot.moveToConfiguration(q)
            if not self.robot.inBounds():
                return -1
            for polygon in self.robot.position:
                for obstacle in self.obstacles:
                    if polygon.doesIntersect(obstacle):
                        return -2
        return 0

    def buildInBetweenConfigurations(self, q_from, q_to):
        # print 'q_from', q_from
        # print 'q_to', q_to
        configurations = []
        q_from_np = np.array(q_from)
        q_to_np = np.array(q_to)
        q_delta = q_to_np - q_from_np
        for i, param in enumerate(q_delta):
            if self.isParamDegreeList[i] == True:
                q_delta[i] = q_delta[i] % 360
        for i in range(1, self.stepsToCheck + 1):
            q = q_from_np + q_delta * float(i) / self.stepsToCheck
            configurations.append(q)
        return configurations

    