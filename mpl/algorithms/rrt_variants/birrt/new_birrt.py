from scipy.spatial import KDTree
import heapq
import numpy as np
import math
import mpl.mplGlobals as mplGlob
from new_rrt import RRTSearcher
from mpl.common import searcher
import pprint
import pdb

class BiRRTSearcher(object):
    def __init__(self, start, goal, helper):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.RRT1 = RRTSearcher(start, goal, helper, extendBackwards = False)
        self.RRT2 = RRTSearcher(goal, start, helper, extendBackwards = True)
        self.foundPath = False
        self.meetingPoint = None

    def run(self):
        for i in range(mplGlob.rrtIterFailLimit):
            success = self.search()
            if success:
                self.foundPath = True
                return True
        self.foundPath = False
        return False

    def search(self):
        for iterNum in range(mplGlob.rrtIterCount):
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

        for i in range(len(pathToReturn) - 1):
            for config in self.helper.generateInBetweenConfigs(pathToReturn[i], pathToReturn[i+1]):
                collisions = self.helper.collisionsAtQ(config)
                # print 'coll', collisions
                if len(collisions) != 0:
                    print 'somehow had collisions when there should be none'
                    pdb.set_trace()
        return pathToReturn


    def getCover(self):
        return []

