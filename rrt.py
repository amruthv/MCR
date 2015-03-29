import random
import numpy as np
from scipy.spatial import KDTree
from collections import defaultdict
import math

class RRT():
    def __init__(self, start, goalBias, goal, generateConfigurationFunction, validateTrajectoryFunction):
        self.start = start
        self.goalBias = goalBias
        self.goal = goal
        self.generateConfigurationFunction = generateConfigurationFunction
        self.trajectoryValidator = validateTrajectoryFunction
        
        self.tree = [start]
        # self.adjacencyList = defaultdict(lambda: set(), {})
        self.cameFrom = {}
        self.maxStepSize = 40

    def growTree(self):
        q_rand = self.generateConfigurationFunction(self.goalBias)
        return self.growTowardsPoint(q_rand)

    def growTowardsPoint(self, q_rand):
        q_near = self.findNearest(q_rand)
        distance = self.distanceBetweenPoints(q_rand, q_near)
        if distance > self.maxStepSize:
            # this difference is too big so only step max step size
            t_d = float(self.maxStepSize) / distance
            stepConfiguration = []
            for paramNum in range(len(q_rand)):
                stepConfiguration.append(q_near[paramNum] + t_d * (q_rand[paramNum] - q_near[paramNum]))
            stepped = stepConfiguration
        else:
            #we can go to q_rand
            stepped = q_rand
        validation =  self.trajectoryValidator(q_near, stepped)
        if validation != 0:
            return (False, None)

        # add it to tree
        self.tree.append(stepped)
        self.cameFrom[tuple(stepped)] = tuple(q_near)

        if self.goal == stepped:
            return (True, stepped)
        return (False, stepped)

    def growUntilFound(self):
        iterNum = 1
        while True:
            if iterNum % 1000 == 0:
                print 'iter', iterNum
            if self.growTree()[0]:
                return
            iterNum += 1

    def findNearest(self, fromPoint):
        best = None
        bestDistance = None
        for otherPoint in self.tree:
            distance = self.distanceBetweenPoints(fromPoint, otherPoint)
            if best == None or distance < bestDistance:
                best = otherPoint
                bestDistance = distance
        return best
    
    #assuming x, y, theta
    def distanceBetweenPoints(self, point1, point2):
        distance = math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) **2 + (point2[2] - point1[2]) **2)
        # distance = math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) **2)
        return distance




