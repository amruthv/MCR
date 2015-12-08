from scipy.spatial import KDTree
import heapq
import numpy as np
import math

class RRTSearcher(object):
    def __init__(self, start, goal, helper, obstaclesToIgnore = set(), shouldDraw = False):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.tree = KDTree([start])
        self.auxillaryArray = []
        self.auxillaryArrayThreshold = 50
        self.cameFrom = {}
        self.obstaclesToIgnore = obstaclesToIgnore
        self.shouldDraw = shouldDraw

    def searchWithRRT(self, numIters = 1000):
        for iterNum in range(numIters):
            qExtended = runIteration()
            print 'qExtended = ', qExtended
            if qExtended == self.goal:
                return True
        return False

    def runIteration(self):
        qRand = self.helper.sampleConfig(self.goal)
        self.rebuildTreeIfNecessary()
        nearestConfig = self.nearestConfig(qRand)
        qExtended = self.extendToward(nearestConfig, qRand)
        self.updateRRTWithNewNode(nearestConfig, qExtended)
        return qExtended

    def updateRRTWithNewNode(self, qNear, qExtended):
        if qExtended is not None:
            self.cameFrom[qExtended] = qNear
            self.auxillaryArray.append(qExtended)


    def rebuildTreeIfNecessary(self):
        if len(self.auxillaryArray) > self.auxillaryArrayThreshold:
            newData = np.vstack([self.tree.data, self.auxillaryArray])
            self.auxillaryArray = []
            self.tree = KDTree(newData)

    def nearestConfig(self, q_rand):
        _, nearestInTreeIndex = self.tree.query(q_rand, 1)
        nearestInTree = tuple(self.tree.data[nearestInTreeIndex])
        nearestInAuxillary, nearestInAuxillaryDistance = self.getNearestFromAuxillary(q_rand)
        if nearestInAuxillary is None:
            nearest = nearestInTree
        else:
            nearestInTreeDistance = self.euclideanDistanceSquared(q_rand, nearestInTree)
            if nearestInTreeDistance < nearestInAuxillaryDistance:
                nearest = nearestInTree
            else:
                nearest = nearestInAuxillary
        return nearest

    def getNearestFromAuxillary(self, q_rand):
        closestSoFar = None
        closestDistance = float('inf')
        for q in self.auxillaryArray:
            distance = self.euclideanDistanceSquared(q, q_rand)
            if distance < closestDistance:
                closestSoFar = q
                closestDistance = distance
        return (closestSoFar, closestDistance)


    # configurations need to have the same dimensions
    def euclideanDistanceSquared(self, q1, q2):
        return sum([(q1[i] - q2[i])**2 for i in range(len(q1))])

    def extendToward(self, closest, sample):
        qPrime = self.helper.stepTowards(closest, sample)
        if set(self.helper.collisionsAtQ(qPrime)) != self.obstaclesToIgnore:
            return None
        for configuration in self.helper.generateInBetweenConfigs(closest, qPrime):
            collisionsAtConfiguration = self.helper.collisionsAtQ(configuration)
            for obstacle in collisionsAtConfiguration:
                if obstacle not in self.obstaclesToIgnore:
                    return None
        return tuple(qPrime)


    def treeSize(self):
        return len(self.tree.data) + len(self.auxillaryArray)
