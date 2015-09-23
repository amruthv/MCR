class MCRHelper():
    # return back list of obstacles that are in collision when robot is at configuration q
    def collisionsAtQ(self, q):
        raise NotImplementedError()

    # return a configuration represented as a list
    def sampleConfig(self):
        raise NotImplementedError()

    # return a list of configurations (as defined above that exclude qFrom and qTo ie (qFrom... qTo) )
    def generateInBetweenConfigs(self, qFrom, qTo):
        raise NotImplementedError()

    # scalar representation of the distance between these configurations
    def distance(self, q1, q2):
        raise NotImplementedError()

