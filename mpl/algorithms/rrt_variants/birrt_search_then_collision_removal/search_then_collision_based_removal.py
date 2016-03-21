import heapq
import numpy as np
import math
from scipy.spatial import KDTree
from new_rrt import RRTSearcher
from mpl.common import covercalculator
from mpl.common.cover import Cover
from mpl.common import searcher
from mpl.common import tlpObstacles
import mpl.mplGlobals as mplGlob
import pdb

#bi rrt implementation that determines obstacles with interest of removing.
class SearchThenCollisionRemovalSearcher(object):
    def __init__(self, start, goal, helper, useTLPObstacles, removalStrategy, memoryFactor = 0.5):
        self.start = start
        self.goal = goal
        self.helper = helper
        obstaclesAtStart = set(helper.collisionsAtQ(start))
        obstaclesAtGoal = helper.collisionsAtQ(goal)
        self.obstaclesAtStartAndGoal = obstaclesAtStart.union(obstaclesAtGoal)
        self.useTLPObstacles = useTLPObstacles
        self.removalStrategy = removalStrategy
        self.memoryFactor = memoryFactor
        self.initializeForIteration()

    def run(self):
        for i in range(mplGlob.rrtIterFailLimit):
            self.initializeForIteration()
            success = self.search()
            if success:
                self.foundPath = True
                return True
        self.foundPath = False
        return False

    def initializeForIteration(self):
        self.obstaclesToIgnore = self.obstaclesAtStartAndGoal.union(set())
        self.obstacleCollisionCounts = {}
        self.RRT1 = RRTSearcher(self.start, self.goal, self.helper, self.obstaclesToIgnore, self.obstacleCollisionCounts, extendBackwards = False)
        self.RRT2 = RRTSearcher(self.goal, self.start, self.helper, self.obstaclesToIgnore, self.obstacleCollisionCounts, extendBackwards = True)
        self.deletedObstacles = {}
        self.meetingPoint = None

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, obstacleRemovalInterval = 30, memoryFactor = 0.5):
        for iterNum in range(2 * mplGlob.rrtIterCount):
            if iterNum > mplGlob.rrtIterCount and iterNum % obstacleRemovalInterval == 0:
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
        if self.useTLPObstacles:
            self.removeTLPObstacle(memoryFactor)
        else:
            self.removeNonTLPObstacle(memoryFactor)

    def removeTLPObstacle(self, memoryFactor):
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
        if len(obstacleRemoveScore) == 0:
            return
        if self.removalStrategy == 'greedy':
            obstacleToRemove, obstacleToRemoveWeight = self.greedyRemoval(obstacleRemoveScore)
        else:
            obstacleToRemove, obstacleToRemoveWeight = self.probabilisticRemoval(obstacleRemoveScore)
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
        obstacleRemoveScore = []
        for obstacle in self.obstacleCollisionCounts:
            if obstacle.getWeight() == float('inf'):
                continue
            scoreForObstacle = self.obstacleCollisionCounts[obstacle] / float(obstacle.getWeight())
            obstacleRemoveScore.append((scoreForObstacle, obstacle))
        if len(obstacleRemoveScore) == 0:
            return
        if self.removalStrategy == 'greedy':
            obstacleToRemove, obstacleToRemoveWeight = self.greedyRemoval(obstacleRemoveScore)
        else:
            obstacleToRemove, obstacleToRemoveWeight = self.probabilisticRemoval(obstacleRemoveScore)
        assert(obstacleToRemove not in self.obstaclesToIgnore)
        self.obstaclesToIgnore.add(obstacleToRemove)
        self.deletedObstacles[obstacleToRemove] = obstacleToRemoveWeight
        del self.obstacleCollisionCounts[obstacleToRemove]
        for obstacle in self.obstacleCollisionCounts:
            self.obstacleCollisionCounts[obstacle] *= memoryFactor

    # there must be an obstacle in obstacleScores
    def greedyRemoval(self, obstacleScores):
        obstacleToRemoveInfo = max(obstacleScores)
        obstacleToRemoveWeight = obstacleToRemoveInfo[0]
        obstacleToRemove = obstacleToRemoveInfo[1]
        return obstacleToRemove, obstacleToRemoveWeight

    def probabilisticRemoval(self, obstacleScores):
        total = sum([score for score, obstacle in obstacleScores])
        obstacles = [obstacle for score, obstacle in obstacleScores]
        probabilities = [float(score)/total for score, _ in obstacleScores]
        obstacleToRemove = np.random.choice(obstacles, p = probabilities)
        return obstacleScores[obstacles.index(obstacleToRemove)][::-1]
    

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
        cc = covercalculator.CoverCalculator(self.helper, self.useTLPObstacles)
        pathCover = Cover(set(), self.useTLPObstacles)
        for i in range(len(trajectory) - 1):
            edgeCoverInclusive = cc.edgeCover(trajectory[i], trajectory[i+1])
            pathCover = pathCover.mergeWith(edgeCoverInclusive)
            if any([obstacle not in self.obstaclesToIgnore for obstacle in pathCover.cover]):
                pdb.set_trace()
        for obstacle in pathCover.cover:
            if obstacle not in self.obstaclesToIgnore:
                print 'gg'
                pdb.set_trace()
        if self.useTLPObstacles:
            return list(pathCover.cover)
        else:
            return pathCover.cover