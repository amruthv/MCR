import heapq
import numpy as np
import math
from scipy.spatial import KDTree
from new_rrt import RRTSearcher
from mpl.algorithms.rrt_variants.birrt_collision_based_removal import birrt_collision_based_removal as birrt
from mpl.common import searcher

#bi rrt implementation that ignores obstacles at the start configuration and goal configuration
class RepetitiveCollisionRemovalSearcher(object):
    def __init__(self, start, goal, helper):
        self.start = start
        self.goal = goal
        self.bestPath = []
        self.bestCover = set()

    def run(self):
        return self.search()

    # want memoryFactor <= 1 used to discount previous weights since removing an obstacle opens up new space
    def search(self, numRepititions = 5):
        bestPath = None
        bestCover = None
        bestCoverScore = float('inf')
        for i in range(numRepititions):
            searcher = birrt.BiRRTCollisionRemovalSearcher(start, goal, helper)
            if searcher.run():
                path = searcher.getPath()
                cover = searcher.getCover()
                coverScore = sum([obstacle.getWeight()
                if bestCover is None or bestCoverScore > coverScore for obstacle in cover]):
                    bestPath = path
                    bestCover = cover
                    bestCoverScore = coverScore
        if bestPath is None or bestPath == []:
            return False
        return True

    def getPath(self):
        return self.bestPath

    def getCover(self):
        return self.bestCover