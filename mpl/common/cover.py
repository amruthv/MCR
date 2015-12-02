from mpl.common import tlpObstacles

class Cover():
    def __init__(self, coverSet, useTLPObstacles):
        self.cover = coverSet
        self.useTLPObstacles = useTLPObstacles
        self.score = self.getCoverScore()

    def getCoverScore(self):
        if self.useTLPObstacles:
            return self.tlpObstacleCoverScore()
        else:
            return self.nonTLPObstacleCoverScore()

    def tlpObstacleCoverScore(self):
        score = 0
        for obstacle in self.cover:
            if obstacle == 'permanent':
                score += tlpObstacles.OBSTACLE_WEIGHTS[obstacle]
            elif obstacle.find('shadow') != -1:
                score += tlpObstacles.OBSTACLE_WEIGHTS['shadow']
            else:
                score += tlpObstacles.OBSTACLE_WEIGHTS['obstacle']
        return score

    def nonTLPObstacleCoverScore(self):
        return sum([obj.getWeight() for obj in self.cover])


    #returns new Cover
    def mergeWith(self, otherCover):
        return Cover(self.cover.union(otherCover.cover), self.useTLPObstacles)

    def __str__(self):
        return str(self.score)

    def __repr__(self):
        return self.__str__()