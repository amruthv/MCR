from scipy.spatial import KDTree
import heapq
import numpy as np
import math

from mpl.common import covercalculator


class RRTSearcher(object):
    def __init__(self, start, goal, helper, obstaclesToIgnore, obstacleCollisionCounts, sim):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.tree = KDTree([start])
        self.auxillaryArray = []
        self.auxillaryArrayThreshold = 50
        self.cameFrom = {}
        self.obstaclesToIgnore = obstaclesToIgnore
        self.obstacleCollisionCounts = obstacleCollisionCounts
        self.sim = sim
        self.cc = covercalculator.CoverCalculator(self.helper, False)


    def searchWithRRT(self, numIters = 1000):
        for iterNum in range(numIters):
            qExtended = runIteration()
            if qExtended == self.goal:
                return True
        return False

    def runIteration(self):
        qRand = self.helper.sampleConfig(self.goal)
        self.rebuildTreeIfNecessary()
        nearestConfig = self.nearestConfig(qRand)
        qExtended = self.extendToward(nearestConfig, qRand)
        # if qExtended is not None:
            # collisions = self.cc.cover(nearestConfig)
            # collisions.mergeWith(self.cc.edgeCover(nearestConfig, qExtended))
            # collisions.mergeWith(self.cc.cover(qExtended))
            # print 'collisions', collisions
        self.updateRRTWithNewNode(nearestConfig, qExtended)
        return qExtended

    def updateRRTWithNewNode(self, qNear, qExtended):
        if qExtended is not None:
            self.sim.drawPoint(qExtended)
            self.sim.drawLine(qNear[0], qNear[1], qExtended[0], qExtended[1])
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
            nearestInTreeDistance = self.helper.distance(q_rand, nearestInTree)
            if nearestInTreeDistance < nearestInAuxillaryDistance:
                nearest = nearestInTree
            else:
                nearest = nearestInAuxillary
        return nearest

    def getNearestFromAuxillary(self, q_rand):
        closestSoFar = None
        closestDistance = float('inf')
        for q in self.auxillaryArray:
            distance = self.helper.distance(q, q_rand)
            if distance < closestDistance:
                closestSoFar = q
                closestDistance = distance
        return (closestSoFar, closestDistance)

    def extendToward(self, closest, sample):
        # print self.obstaclesToIgnore
        qPrime = self.helper.stepTowards(closest, sample)
        # print 'closest', closest
        # print 'sample', sample
        for configuration in self.helper.generateInBetweenConfigs(closest, qPrime):
            # print configuration
            collisionsAtConfiguration = self.helper.collisionsAtQ(configuration)
            collisionsToNotIgnore = set()
            for obstacle in collisionsAtConfiguration:
                if obstacle not in self.obstaclesToIgnore:
                    collisionsToNotIgnore.add(obstacle)
            # print 'collisions found', collisionsToNotIgnore
            if len(collisionsToNotIgnore) != 0:
                # print 'hi'
                for collision in collisionsToNotIgnore:
                    currentCollisionCountForObstacle = self.obstacleCollisionCounts.get(collision, 0)
                    # print 'collision with obstacle', collision
                    # print 'currentCollisionCountForObstacle', currentCollisionCountForObstacle 
                    self.obstacleCollisionCounts[collision] = currentCollisionCountForObstacle + 1
                return None
        # print 'no collisions!!!!!!!'
        return tuple(qPrime)

    def treeSize(self):
        return len(self.tree.data) + len(self.auxillaryArray)