class Cover():
    def __init__(self, coverSet):
        self.cover = coverSet
        self.score = sum([obj.weight for obj in self.cover])

    #returns new Cover
    def mergeWith(self, otherCover):
        return Cover(self.cover.union(otherCover.cover))