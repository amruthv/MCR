import math
import random
import numpy as np
from mpl.common import tlpObstacles
from mpl.common import cover
from mpl.algorithms.rrt_variants.birrt_collision_based_removal import birrt_collision_based_removal as birrt


#bi rrt implementation that ignores obstacles at the start configuration and goal configuration
class BiRRTCollisionRemovalAddingSearcher(object):
    def __init__(self, start, goal, helper, useTLPObjects):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.bestPath = []
        self.bestCover = set()
        self.useTLPObjects = useTLPObjects

    def run(self):
        return self.search()

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, numAddAttempts = 5, memoryFactor = 0.4):
        searcher = birrt.BiRRTCollisionRemovalSearcher(self.start, self.goal, self.helper, self.useTLPObjects)
        if not searcher.search(memoryFactor = memoryFactor):
            return False
        searcherCover = searcher.getCover()
        for i in range(numAddAttempts):
            deletedObstaclesAndWeights = searcher.deletedObstacles
            if len(deletedObstaclesAndWeights) == 0:
                return True
            deletedObstaclesAsList = deletedObstaclesAndWeights.items()
            obstaclesToAdd = self.selectObstaclesToAdd(deletedObstaclesAsList)
            obstaclesToIgnore = self.buildIgnoreObstaclesSet(deletedObstaclesAsList, obstaclesToAdd)
            newSearcher = birrt.BiRRTCollisionRemovalSearcher(self.start, self.goal, self.helper, self.useTLPObjects, obstaclesToIgnore)
            newSearcherSucceeded = newSearcher.search(memoryFactor = memoryFactor)
            print type(obstaclesToAdd)
            print len(obstaclesToAdd)
            print type(obstaclesToAdd[0])
            if not newSearcherSucceeded or \
                any([obstacleToAdd in newSearcher.deletedObstacles for obstacleToAdd in obstaclesToAdd]):
                continue
            newCover = newSearcher.getCover()
            if cover.Cover(newCover, self.useTLPObjects).score < cover.Cover(searcherCover, self.useTLPObjects).score:
                searcher = newSearcher
                searcherCover = newCover
        self.bestPath = searcher.getPath()
        self.bestCover = searcher.getCover()            

    #deletedObstaclesAndWeights must be nonempty
    def selectObstaclesToAdd(self, deletedObstaclesAsList):
        collisionCountsList = [x[1] for x in deletedObstaclesAsList]
        obstacles = [x[0] for x in deletedObstaclesAsList]
        probabilities = self.determineProbabilities(collisionCountsList)
        obstaclesToAdd = [ np.random.choice(obstacles, p = probabilities) ]
        print type(obstaclesToAdd)
        if self.useTLPObjects:
            if obstaclesToAdd[0].find('shadow') != -1:
                companionObstacle = tlpObstacles.getObstacleFromShadow(obstaclesToAdd[0])
                obstaclesToAdd.append(companionObstacle)
        return obstaclesToAdd

    def determineProbabilities(self, collisionCountsList):
        referenceNumber = float(collisionCountsList[0])
        scalingFactors = []
        for numberCollisions in collisionCountsList:
            scalingFactors.append(referenceNumber / numberCollisions)
        referenceProbability = 1.0 / sum(scalingFactors)
        probabilities = [referenceProbability * scalingFactor for scalingFactor in scalingFactors]
        return probabilities

    def buildIgnoreObstaclesSet(self, deletedObstaclesAsList, obstaclesToAdd):
        obstaclesToIgnore = set()
        for obstacle, _ in deletedObstaclesAsList:
            if obstacle not in obstaclesToAdd:
                obstaclesToIgnore.add(obstacle)
        return obstaclesToIgnore            

    def getPath(self):
        return self.bestPath

    def getCover(self):
        return self.bestCover    