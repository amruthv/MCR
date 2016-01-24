import heapq
import numpy as np
import math
from scipy.spatial import KDTree
from new_rrt import RRTSearcher
from mpl.common import covercalculator
from mpl.common.cover import Cover
from mpl.common import searcher
from mpl.common import tlpObstacles
import pdb

#bi rrt implementation that determines obstacles with interest of removing.
class BiRRTIgnoreObstaclesSearcher(object):
    def __init__(self, start, goal, helper, useTLPObjects):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.RRT1 = RRTSearcher(start, goal, helper, useTLPObjects, extendBackwards = False)
        self.RRT2 = RRTSearcher(goal, start, helper, useTLPObjects, extendBackwards = True)
        self.meetingPoint = None
        self.useTLPObjects = useTLPObjects

    def run(self):
        print 'in ignore all birrt run'
        self.foundPath = self.search()
        return self.foundPath

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, numIters = 500):
        for iterNum in range(numIters):
            qExtended = self.RRT1.runIteration()
            if qExtended is not None:
                self.RRT2.rebuildTreeIfNecessary()
                qNearest = self.RRT2.nearestConfig(qExtended)
                if qNearest == qExtended:
                    self.meetingPoint = qExtended
                    return True 
                qExtendedTree2 = self.RRT2.extendToward(qNearest, qExtended)
                if qExtendedTree2 is not None:
                    self.RRT2.updateRRTWithNewNode(qNearest, qExtendedTree2)
                    if qExtendedTree2 == qExtended:
                        self.meetingPoint = qExtended
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

        pathFromStart = searcher.reconstructPath(tree1.cameFrom, self.meetingPoint)
        pathFromGoal = searcher.reconstructPath(tree2.cameFrom, self.meetingPoint)
        if self.meetingPoint == self.goal:
            pathToReturn = pathFromStart
        elif self.meetingPoint == self.start:
            pathToReturn = pathFromGoal[::-1]
        else:
            pathFromMeetingToGoal = pathFromGoal[:-1][::-1]
            pathFromMeetingToGoal = pathFromMeetingToGoal[:-1] + [self.goal]   
            pathToReturn = pathFromStart + pathFromMeetingToGoal
        pathToReturn = [self.start] + pathToReturn[1:-1] + [self.goal]
        return pathToReturn

    def getCover(self):
        trajectory = self.getPath()
        cc = covercalculator.CoverCalculator(self.helper, self.useTLPObjects)
        pathCover = Cover(set(), self.useTLPObjects)
        for i in range(len(trajectory) - 1):
            edgeCoverInclusive = cc.edgeCover(trajectory[i], trajectory[i+1])
            pathCover = pathCover.mergeWith(edgeCoverInclusive)
        return list(pathCover.cover)
