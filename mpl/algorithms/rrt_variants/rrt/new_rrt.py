from scipy.spatial import KDTree
import pdb
import heapq
import numpy as np
import math
from mpl.common import searcher

class RRTSearcher(object):
    def __init__(self, start, goal, helper, obstaclesToIgnore = set(), shouldDraw = False):
        if not start or not goal:
            pdb.set_trace()
        print 'in rrt algorithm initializer'
        print 'start =', start
        print 'goal =', goal
        self.start = start
        self.goal = goal
        self.helper = helper
        self.tree = KDTree([start])
        self.auxillaryArray = []
        self.auxillaryArrayThreshold = 50
        self.cameFrom = {}
        self.obstaclesToIgnore = obstaclesToIgnore
        self.shouldDraw = shouldDraw
        self.foundPath = False

    def run(self):
        if self.searchWithRRT():
            self.foundPath = True
        return self.foundPath

    def searchWithRRT(self, numIters = 1000):
        for iterNum in range(numIters):
            qExtended = self.runIteration()
            # print 'qExtended=', qExtended
            if qExtended == self.goal:
                print 'found path to goal'
                return True
        print 'couldn\'t find path to goal'
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
        print 'closest', closest
        print 'sample', sample
        print 'goal', self.goal
        qPrime = self.helper.stepTowards(closest, sample)
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

    def getCover(self):
        return set()

    def getPath(self):
        if self.foundPath:
            return searcher.reconstructPath(self.cameFrom, self.goal)
        return []