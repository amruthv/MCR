from scipy.spatial import KDTree
import heapq
import numpy as np
import math
from new_rrt import RRTSearcher
from mpl.common import searcher
import pprint

class BiRRTSearcher(object):
    def __init__(self, start, goal, helper):
        self.start = start
        self.goal = goal
        self.RRT1 = RRTSearcher(start, goal, helper)
        self.RRT2 = RRTSearcher(goal, start, helper)
        self.foundPath = False

    def run(self):
        self.foundPath = self.search()

    def search(self, numIters = 1000):
        for iterNum in range(numIters):
            qExtended = self.RRT1.runIteration()
            if qExtended is not None:
                self.RRT2.rebuildTreeIfNecessary()
                qNearest = self.RRT2.nearestConfig(qExtended)
                if qNearest == qExtended:
                    return True 
                qExtendedTree2 = self.RRT2.extendToward(qNearest, qExtended)            
                if qExtendedTree2 is not None:
                    self.RRT2.updateRRTWithNewNode(qNearest, qExtended)
                    if qExtendedTree2 == qExtended:
                        self.foundPath = True
                        return True
            if self.RRT1.treeSize() > self.RRT2.treeSize():
                self.RRT1, self.RRT2 = self.RRT2, self.RRT1
        return False

    def getPath(self):
        if not self.foundPath:
            return []
        if self.RRT1.goal == self.goal:
            tree1 = self.RRT1
            tree2 = self.RRT2
        else:
            tree1 = self.RRT2
            tree2 = self.RRT1
        # find the meeting point of the two trees
        commonKeys = set(tree1.cameFrom.keys()).intersection(set(tree2.cameFrom.keys()))
        assert(len(commonKeys) >= 1)
        meetingPoint = commonKeys.pop()
        pathFromStart = searcher.reconstructPath(tree1.cameFrom, meetingPoint)
        pathFromGoal = searcher.reconstructPath(tree2.cameFrom, meetingPoint)
        trajectory = pathFromStart + pathFromGoal[:-1][::-1]
        return trajectory

    def getCover(self):
        return set()

