import math
import random
import numpy as np
from mpl.common import cover
from mpl.algorithms.rrt_variants.birrt_collision_based_removal import birrt_collision_based_removal as birrt


#bi rrt implementation that ignores obstacles at the start configuration and goal configuration
class BiRRTCollisionRemovalAddingSearcher(object):
    def __init__(self, start, goal, helper):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.bestPath = []
        self.bestCover = set()

    def run(self):
        return self.search()

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, numAddAttempts = 5, memoryFactor = 0.4):
        searcher = birrt.BiRRTCollisionRemovalSearcher(self.start, self.goal, self.helper)
        if not searcher.search(memoryFactor = memoryFactor):
            return False
        searcherCover = searcher.getCover()
        for i in range(numAddAttempts):
            deletedObstaclesAndWeights = searcher.deletedObstacles
            if len(deletedObstaclesAndWeights) == 0:
                return True
            deletedObstaclesAsList = eletedObstaclesAndWeights.items()
            obstacleToAdd = self.selectObstacleToAdd(deletedObstaclesAsList)
            obstaclesToIgnore = self.buildIgnoreObstaclesSet(deletedObstaclesAsList, obstacleToAdd)
            newSearcher = birrt.BiRRTCollisionRemovalSearcher(self.start, self.goal, self.helper, obstaclesToIgnore)
            if not newSearcher.search(memoryFactor = memoryFactor) or obstacleToAdd in newSearcher.deletedObstacles:
                continue
            newCover = newSearcher.getCover()
            if cover.Cover(newCover).score < cover.Cover(searcherCover).score:
                searcher = newSearcher
                searcherCover = newCover
        self.bestPath = searcher.getPath()
        self.bestCover = searcher.getCover()            

    #deletedObstaclesAndWeights must be nonempty
    def selectObstacleToAdd(self, deletedObstaclesAsList):
        collisionCountsList = [x[1] for x in deletedObstaclesAsList]
        obstacles = [x[0] for x in deletedObstaclesAsList]
        probabilities = self.determineProbabilities(collisionCountsList)
        return np.random.choice(obstacles, probabilities)

    def determineProbabilities(collisionCountsList):
        referenceNumber = float(collisionCountsList[0])
        scalingFactors = []
        for i in collisionCountsList:
            scalingFactors.append(referenceNumber / collisionCountsList[i])
        referenceProbability = 1.0 / sum(scalingFactors)
        probabilities = [referenceProbability * scalingFactor for scalingFactor in scalingFactors]
        return probabilities



    def buildIgnoreObstaclesSet(deletedObstaclesAsList, obstacleToAdd):
        obstaclesToIgnore = set()
        for obstacle, _ in deletedObstaclesAsList:
            if obstacle != obstacleToAdd:
                obstaclesToIgnore.add(obstacle)
        return obstaclesToIgnore            

    def getPath(self):
        return self.bestPath

    def getCover(self):
        return self.bestCover    