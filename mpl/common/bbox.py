from shapely.geometry import Polygon
import pdb

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
        self.bboxCorners = [(self.minX, self.minY), (self.maxX, self.minY), (self.maxX, self.maxY), (self.minX, self.maxY)]

    def intersectsBBox(self, other):
        selfPolygon = Polygon(self.bboxCorners)
        otherPolygon = Polygon(other.bboxCorners)
        overlaps = selfPolygon.overlaps(otherPolygon)
        intersects = selfPolygon.intersects(otherPolygon)
        if overlaps and not intersects:
            pdb.set_trace()
        return intersects or overlaps
