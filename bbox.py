class BBox():
    #points is [(x,y), (x1,y1),...]
    def __init__(self, points):
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        self.points = points
        self.minX = min(xs)
        self.maxX = max(xs)
        self.minY = min(ys)
        self.maxY = max(ys)

    def intersectsBBox(self, other):
        for pt in self.points:
            if other.minX < pt[0] < other.maxX and other.minY < pt[1] < other.maxY:
                return True
        return False
