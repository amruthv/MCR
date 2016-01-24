from scipy.spatial import KDTree
import heapq
import numpy as np
import math
import pdb

from mpl.common import covercalculator


class RRTSearcher(object):
    def __init__(self, start, goal, helper, useTLPObjects, extendBackwards):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.tree = KDTree([start])
        self.auxillaryArray = []
        self.auxillaryArrayThreshold = 50
        self.cameFrom = {}
        self.useTLPObjects = useTLPObjects
        self.extendBackwards = extendBackwards

    def runIteration(self):
        qRand = self.helper.sampleConfig(self.goal)
        self.rebuildTreeIfNecessary()
        nearestConfig = self.nearestConfig(qRand)
        qExtended = self.extendToward(nearestConfig, qRand)
        self.updateRRTWithNewNode(nearestConfig, qExtended)
        return qExtended

    def updateRRTWithNewNode(self, qNear, qExtended):
        if qNear == qExtended:
            pdb.set_trace()
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
        qPrime = self.helper.stepTowards(closest, sample)
        if self.extendBackwards:
            generator = self.helper.generateInBetweenConfigs(qPrime, closest)
        else:
            generator = self.helper.generateInBetweenConfigs(closest, qPrime)
        for configuration in generator:
            collisionsAtConfiguration = self.helper.collisionsAtQ(configuration)
            if self.useTLPObjects:
                for obstacle in collisionsAtConfiguration:
                    if obstacle == 'permanent':
                        return None
        return tuple(qPrime)

    def treeSize(self):
        return len(self.tree.data) + len(self.auxillaryArray)