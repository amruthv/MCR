from shapely.geometry import Polygon

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
        self.bboxCorners = [(self.minX, self.minY), (self.minX, self.maxY), (self.maxX, self.minY), (self.maxX, self.maxY)]

    def intersectsBBox(self, other):
        selfPolygon = Polygon(self.bboxCorners)
        otherPolygon = Polygon(other.bboxCorners)
        return selfPolygon.overlaps(otherPolygon)
