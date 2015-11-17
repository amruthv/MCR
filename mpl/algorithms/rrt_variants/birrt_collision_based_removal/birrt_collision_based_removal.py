import heapq
import numpy as np
import math
from scipy.spatial import KDTree
from new_rrt import RRTSearcher
from mpl.common import covercalculator
from mpl.common import searcher

#bi rrt implementation that ignores obstacles at the start configuration and goal configuration
class BiRRTCollisionRemovalSearcher(object):
    def __init__(self, start, goal, helper):
        self.start = start
        self.goal = goal
        self.helper = helper
        obstaclesAtStart = helper.collisionsAtQ(start)
        obstaclesAtGoal = helper.collisionsAtQ(goal)
        obstaclesAtStartAndGoal = obstaclesAtStart.union(obstaclesAtGoal)
        self.obstaclesToIgnore = obstaclesAtStartAndGoal
        self.obstacleCollisionCounts = {}
        self.RRT1 = RRTSearcher(start, goal, helper, self.obstaclesToIgnore, self.obstacleCollisionCounts)
        self.RRT2 = RRTSearcher(goal, start, helper, self.obstaclesToIgnore, self.obstacleCollisionCounts)
        self.meetingPoint = None

    def run(self):
        return self.search()

    def search(self, numIters = 1000, obstacleRemovalInterval = 100):
        for iterNum in range(numIters):
            if iterNum > 0 and iterNum % obstacleRemovalInterval == 0:
                self.selectObstacleToRemove()
            qExtended = self.RRT1.runIteration()
            if qExtended is not None:
                self.RRT2.rebuildTreeIfNecessary()
                qNearest = self.RRT2.nearestConfig(qExtended)
                if qNearest == qExtended:
                    self.meetingPoint = qExtended
                    return True 
                qExtendedTree2 = self.RRT2.extendToward(qNearest, qExtended)
                if qExtendedTree2 is not None:
                    self.RRT2.updateRRTWithNewNode(qNearest, qExtended)
                    if qExtendedTree2 == qExtended:
                        self.meetingPoint = qExtended
                        return True
            if self.RRT1.treeSize() > self.RRT2.treeSize():
                self.RRT1, self.RRT2 = self.RRT2, self.RRT1
        return False

    def selectObstacleToRemove(self):
        obstacleRemoveScore = []
        for obstacle in self.obstacleCollisionCounts:
            scoreForObstacle = obstacle.getWeight() * self.obstacleCollisionCounts[obstacle]
            obstacleRemoveScore.append((scoreForObstacle, obstacle))
        obstacleToRemove = min(obstacleRemoveScore)[0]
        assert(obstacleToRemove not in self.obstaclesToIgnore)
        self.obstaclesToIgnore.add(obstacleToRemove)
        del self.obstacleCollisionCounts[obstacle]


    def getPath(self):
        if self.meetingPoint is None:
            return []
        if self.RRT1.goal == self.goal:
            tree1 = self.RRT1
            tree2 = self.RRT2
        else:
            tree1 = self.RRT2
            tree2 = self.RRT1
        # find the meeting point of the two trees
        pathFromStart = searcher.reconstructPath(tree1.cameFrom, self.meetingPoint)
        pathFromGoal = searcher.reconstructPath(tree2.cameFrom, self.meetingPoint)
        trajectory = pathFromStart + pathFromGoal[::-1]
        pathCover = self.getCoverOfPath(trajectory)
        return (trajectory, pathCover)

    def getCover(self, trajectory):
        cc = covercalculator.CoverCalculator(self.helper)
        cover = cc.cover(self.start)
        for i in range(len(trajectory) - 1):
            edgeCover = cc.edgeCover(trajectory[i], trajectory[i+1])
            cover = cover.mergeWith(edgeCover)
            cover = cover.mergeWith(cc.cover(trajectory[i+1]))
        return cover.cover