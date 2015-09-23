from mcr import MCRPlanner
from mcrhelper import MCRHelper

# start is a list/tuple of parameters specifying configuration
# goal is a list/tuple of parameters specifying configuration
#
def runMCMR(start, goal, numIterations = 50):
    helper = MyMCRHelper()
    mcr = MCRPlanner(start, goal, helper)
    min_cover_weight = mcr.discreteMCR()
    path = mcr.getBestPath()
    best_cover = mcr.getCoverOfBestPath()
    print 'best cover: ' + best_cover
    print 'path: ' + path
    return best_cover


class MyMCRHelper(MCRHelper):

    #return objects that you collides at this configuration q -- must be hashable.
    def collisionsAtQ(self, q):
        raise NotImplementedError()

    # return a configuration represented as a list or tuple
    def sampleConfig(self):
        raise NotImplementedError()

    # return a list of configurations (as defined above that exclude qFrom and qTo ie (qFrom... qTo) )
    def generateInBetweenConfigs(self, qFrom, qTo):
        raise NotImplementedError()

    # scalar representation of the distance between these configurations
    def distance(self, q1, q2):
        raise NotImplementedError()
