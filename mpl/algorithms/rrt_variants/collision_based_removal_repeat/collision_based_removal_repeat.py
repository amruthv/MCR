import heapq
import numpy as np
import math
from mpl.common import cover
from mpl.algorithms.rrt_variants.birrt_collision_based_removal import birrt_collision_based_removal as birrt
import pdb

#bi rrt implementation that ignores obstacles at the start configuration and goal configuration
class RepetitiveCollisionRemovalSearcher(object):
    def __init__(self, start, goal, helper, useTLPObstacles, removalStrategy):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.bestPath = []
        self.bestCover = set()
        self.useTLPObstacles = useTLPObstacles
        self.removalStrategy = removalStrategy

    def run(self):
        return self.search()

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, numRepititions = 10):
        bestPath = []
        bestCover = None
        for i in range(numRepititions):
            searcher = birrt.BiRRTCollisionRemovalSearcher(self.start, self.goal, self.helper, self.useTLPObstacles, self.removalStrategy)
            if searcher.run():
                iterationPath = searcher.getPath()
                iterationCover = searcher.getCover()
                coverObj = cover.Cover(set(iterationCover), self.useTLPObstacles)
                if iterationPath != [] and (bestCover is None or bestCover.score > coverObj.score):
                    bestPath = iterationPath
                    bestCover = coverObj
        self.bestPath = bestPath
        if self.bestPath == []:
            self.bestCover = set()
            return False
        else:
            self.bestCover = bestCover.cover
            return True

    def getPath(self):
        return self.bestPath

    def getCover(self):
        if self.useTLPObstacles:
            return list(self.bestCover.cover)
        else:
            return self.bestCover