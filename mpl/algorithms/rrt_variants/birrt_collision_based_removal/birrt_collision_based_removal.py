import heapq
import numpy as np
import math
from scipy.spatial import KDTree
from new_rrt import RRTSearcher
from mpl.common import covercalculator
from mpl.common import searcher
from mpl.common import tlpObstacles
import pdb

#bi rrt implementation that determines obstacles with interest of removing.
class BiRRTCollisionRemovalSearcher(object):
    def __init__(self, start, goal, helper, useTLPObjects, obstaclesToIgnore = set(), sim = None):
        self.start = start
        self.goal = goal
        self.helper = helper
        obstaclesAtStart = set(helper.collisionsAtQ(start))
        obstaclesAtGoal = helper.collisionsAtQ(goal)
        obstaclesAtStartAndGoal = obstaclesAtStart.union(obstaclesAtGoal)
        self.obstaclesToIgnore = obstaclesToIgnore.union(obstaclesAtStartAndGoal)
        print self.obstaclesToIgnore
        self.obstacleCollisionCounts = {}
        self.RRT1 = RRTSearcher(start, goal, helper, self.obstaclesToIgnore, self.obstacleCollisionCounts, sim)
        self.RRT2 = RRTSearcher(goal, start, helper, self.obstaclesToIgnore, self.obstacleCollisionCounts, sim)
        self.meetingPoint = None
        self.useTLPObjects = useTLPObjects
        self.deletedObstacles = {}
        self.cc = covercalculator.CoverCalculator(self.helper, self.useTLPObjects)
        if sim is not None:
            self.sim = sim
            startId = self.sim.drawPoint(start)
            goalId = self.sim.drawPoint(goal)

    def run(self):
        return self.search()

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, numIters = 1000, obstacleRemovalInterval = 200, memoryFactor = 1):
        for iterNum in range(numIters):
            # print 'iterNum=', iterNum
            # print self.obstacleCollisionCounts
            if iterNum > 0 and iterNum % obstacleRemovalInterval == 0:
                self.selectObstacleToRemove(memoryFactor)
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

    def selectObstacleToRemove(self, memoryFactor):
        if self.useTLPObjects:
            print 'using tlp obstacles'
            self.removeTLPObstacle(memoryFactor)
        else:
            print 'not using tlp obstacles'
            self.removeNonTLPObstacle(memoryFactor)

    def selectTLPObstacle(memoryFactor):
        obstacleRemoveScore = []
        for obstacle in self.obstacleCollisionCounts:
            # don't want to remove a immovable obstacle
            if obstacle == 'permanent':
                continue
            if obstacle.find('shadow') != -1:
                weight = tlpObstacles.OBSTACLE_WEIGHTS['shadow']
            else:
                weight = tlpObstacles.OBSTACLE_WEIGHTS['obstacle']
            scoreForObstacle = self.obstacleCollisionCounts[obstacle] / float(weight)
            obstacleRemoveScore.append((scoreForObstacle, obstacle))
        obstacleToRemoveInfo = max(obstacleRemoveScore)
        obstacleToRemoveWeight = obstacleRemoveInfo[0]
        obstacleToRemove = obstacleRemoveInfo[1]
        isObstacle = obstacleToRemove.find('shadow') == -1
        assert(obstacleToRemove not in self.obstaclesToIgnore)
        self.obstaclesToIgnore.add(obstacleToRemove)
        self.deletedObstacles[obstacleToRemove] = obstacleToRemoveWeight
        del self.obstacleCollisionCounts[obstacleToRemove]
        if isObstacle:
            companionShadow = tlpObstacles.getShadowFromObstacle(obstacleToRemove)
            if companionShadow not in self.obstaclesToIgnore:
                self.obstaclesToIgnore.add(companionShadow)
                self.deletedObstacles[companionShadow] = \
                    [score for score, obstacle in obstacelRemoveScore if obstacle == companionShadow][0]
                del self.obstacleCollisionCounts[companionShadow]
        for obstacle in self.obstacleCollisionCounts:
            self.obstacleCollisionCounts[obstacle] *= memoryFactor

    def removeNonTLPObstacle(self, memoryFactor):
        print 'REMOVING AN OBSTACLE'
        obstacleRemoveScore = []
        print 'collision counts', self.obstacleCollisionCounts
        for obstacle in self.obstacleCollisionCounts:
            if obstacle.getWeight() == float('inf'):
                continue
            scoreForObstacle = self.obstacleCollisionCounts[obstacle] / float(obstacle.getWeight())
            obstacleRemoveScore.append((scoreForObstacle, obstacle))
        obstacleToRemoveInfo = max(obstacleRemoveScore)
        obstacleToRemoveWeight = obstacleToRemoveInfo[0]
        obstacleToRemove = obstacleToRemoveInfo[1]
        print 'REMOVING OBSTACLE', obstacleToRemove
        assert(obstacleToRemove not in self.obstaclesToIgnore)
        self.obstaclesToIgnore.add(obstacleToRemove)
        self.deletedObstacles[obstacleToRemove] = obstacleToRemoveWeight
        del self.obstacleCollisionCounts[obstacleToRemove]
        for obstacle in self.obstacleCollisionCounts:
            self.obstacleCollisionCounts[obstacle] *= memoryFactor


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
        return trajectory

    def getCover(self):
        trajectory = self.getPath()
        cc = covercalculator.CoverCalculator(self.helper, self.useTLPObjects)
        cover = cc.cover(self.start)
        for i in range(len(trajectory) - 1):
            edgeCover = cc.edgeCover(trajectory[i], trajectory[i+1])
            cover = cover.mergeWith(edgeCover)
            cover = cover.mergeWith(cc.cover(trajectory[i+1]))
            if any([obstacle not in self.obstaclesToIgnore for obstacle in cover.cover ]):
                pdb.set_trace()
        for obstacle in cover.cover:
            if obstacle not in self.obstaclesToIgnore:
                print 'gg'
                pdb.set_trace()
        return cover.cover