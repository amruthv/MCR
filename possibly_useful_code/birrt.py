from rrt import RRT
import numpy as np

class BiRRT():
    def __init__(self, start, goalBias, goal, generateConfigurationFunction, validateTrajectoryFunction):
        self.startRRT = RRT(start, goalBias, goal, generateConfigurationFunction, validateTrajectoryFunction)
        self.goalRRT = RRT(goal, goalBias, start, generateConfigurationFunction, validateTrajectoryFunction)
        self.validateTrajectory = validateTrajectoryFunction
        self.maxTreeConnectionDistance = 40

    def growUntilFound(self):
        while True:
            success, addedPoint = self.startRRT.growTree()
            if success:
                return addedPoint
            else:
                if addedPoint is not None:
                    # lets grow tree 2 towards the stepped point
                    success, goalAddedPoint = self.goalRRT.growTowardsPoint(addedPoint)
                    if goalAddedPoint == addedPoint:
                        #we met so return
                        return addedPoint

    # def growTrees(self):
    #     connectivity = self.checkTreeConnectivity():
    #     if connectivity is not None:
    #         self.startRRT.adjacencyList[connectivity[0]].add(connectivity[1])
    #         return True
    #     self.startRRT.growTree()
    #     self.goalRRT.growTree()
    #     return False

    # def checkTreeConnectivity(self):
    #     for point in self.startRRT.data:
    #         for otherPoint in self.goalRRT.data:
    #             if np.linalg.norm(point - otherPoint) <= self.maxTreeConnectionDistance:
    #                 # we can connect these two points in the different trees by distance now check collision
    #                 if self.validateTrajectory(point, otherPoint):
    #                     return (point, otherPoint)
    #     return None