class Robot():
    #polygonArr is a array of array of points (not polygon objects or point objects) e.g [[[1,2],[3,4]], [[5,5],[6,6]]] 
    def __init__(self, polygonArr):
        raise NotImplementedError()

    # keep generating them until one is valid
    def generateRandomConfiguration(self):
        raise NotImplementedError()

    def distance(self, q1, q2):
        raise NotImplementedError()
