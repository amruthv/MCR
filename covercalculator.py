import numpy as np
from cover import Cover

class CoverCalculator():
    def __init__(self, mcrhelper):
        self.mcrhelper = mcrhelper

    #check that the configurations are in bounds elsewhere, here we assume they are. only checks configurations between the two not the endpoints
    def edgeCover(self, q_from, q_to):
        #make empty cover and build it up
        edge_cover = Cover(set())
        configurationsToCheck = self.mcrhelper.generateInBetweenConfigs(q_from, q_to)
        for q in configurationsToCheck:
            coverQ = self.cover(q)
            edge_cover = edge_cover.mergeWith(coverQ)         
        return edge_cover

    def cover(self, q):
        coverQ = set()
        collisions = self.mcrhelper.collisionsAtQ(q)
        for collision in collisions:
            coverQ.add(collision)
        return Cover(coverQ)




    