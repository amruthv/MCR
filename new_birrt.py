from scipy.spatial import KDTree
import heapq
import numpy as np
import math
from new_rrt import RRTSearcher

class BiRRTSearcher(object):
    def __init__(self, start, goal, helper):
        self.start = start
        self.goal = goal
        self.RRT1 = RRTSearcher(start, goal, helper)
        self.RRT2 = RRTSearcher(goal, start, helper)
        self.cameFrom = {}

    def search(self, numIters = 1000):
        for iterNum in range(numIters):
            qExtended = self.RRT1.runIteration()
            if qExtended is not None:
                self.RRT2.rebuildTreeIfNecessary()
                qNearest = self.RRT2.nearestConfig(qExtended)
                qExtendedTree2 = self.RRT2.extendToward(qNearest, qExtended)
                if qExtendedTree2 is not None:
                    self.RRT2.updateRRTWithNewNode(qNearest, qExtended)
                    if qExtendedTree2 == qExtended:
                        return True
            if self.RRT1.treeSize() > self.RRT2.treeSize():
                self.RRT1, self.RRT2 = self.RRT2, self.RRT1
        return False

    def path(self):
        if self.RRT1.goal == self.goal:
            tree1 = self.RRT1
            tree2 = self.RRT2
        else:
            tree1 = self.RRT2
            tree2 = self.RRT1
        reversedTree2CameFrom = dict((v, k) for k, v in tree2.cameFrom.iteritems())
        entireCameFrom = tree1.cameFrom.copy()
        entireCameFrom.update(reversedTree2CameFrom)
        return entireCameFrom
