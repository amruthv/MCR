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
            qExtended = self.runIteration()
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

    def extendToward(self, closest, sample, delta = 400, complicated = False):
        if complicated:
            return self.complicatedExtendToward(closest, sample, delta)
        else:
            return self.simpleExtendToward(closest, sample, delta)

    def simpleExtendToward(self, closest, sample, delta):
        scaleFactor = min(delta / math.sqrt(self.euclideanDistanceSquared(closest, sample)), 1)
        scaledVector = scaleFactor * (np.array(sample) - np.array(closest))
        qPrime = np.array(closest) + scaledVector
        configurationsToCheck = self.helper.generateInBetweenConfigs(closest, qPrime)
        configurationsToCheck.append(qPrime)
        collisions = set()
        for configuration in configurationsToCheck:
            collisionsAtConfiguration = self.helper.collisionsAtQ(configuration)
            for obstacle in collisionsAtConfiguration:
                if obstacle not in self.obstaclesToIgnore:
                    collisions.add(obstacle)
        if len(collisions) == 0:
            return tuple(qPrime)
        return None

    def treeSize(self):
        return len(self.tree.data) + len(self.auxillaryArray)


    def complicatedExtendToward(self, closest, sample, delta, bisectionLimit = 4):
        scaleFactor = min(delta / math.sqrt(self.euclideanDistanceSquared(closest, sample)), 1)
        scaledVector = scaleFactor * (np.array(sample) - np.array(closest))
        qPrime = np.array(closest) + scaledVector
        bisectionCount = 0
        collisionCheckMap = {}
        while True:
            configurationsToCheck = self.helper.generateInBetweenConfigs(closest, qPrime)
            configurationsToCheck.append(qPrime)
            configsAreCollisionFree = True
            for configurationToCheck in configurationsToCheck:
                tupleisedConfiguration = tuple(configurationToCheck)
                if tupleisedConfiguration in collisionCheckMap:
                    numCollisionsAtConfiguration = collisionCheckMap[tupleisedConfiguration]             
                else:
                    obstacleCollisions = self.helper.collisionsAtQ(configurationToCheck)
                    numCollisionsAtConfiguration = len(obstacleCollisions)
                    collisionCheckMap[tupleisedConfiguration] = numCollisionsAtConfiguration
                if numCollisionsAtConfiguration != 0:
                    configsAreCollisionFree = False
                    break
            
            if configsAreCollisionFree:
                return tuple(qPrime)
            elif bisectionCount < bisectionLimit:
                bisectionCount += 1
                qPrime = tuple(0.5 * (qPrime + np.array(closest)))
            else:
                return None