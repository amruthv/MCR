import numpy as np
import math
from matplotlib.path import Path

class Point():
    def __init__(self, x, y):
        self.arr = np.array([x,y])
        self.x = x
        self.y = y
    def __str__(self):
        return "({0},{1})".format(self.x, self.y)
    def __repr__(self):
        return self.__str__()
    def add(self, dx,dy):
        return Point(self.x + dx, self.y + dy)
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class Edge():
    def __init__(self, point1, point2):
        self.fromPoint = point1.arr
        self.toPoint = point2.arr
    def __str__(self):
        return self.fromPoint.__str__() + " -> " + self.toPoint.__str__()
    def __repr__(self):
        return self.__str__()

    def getReverseEdge(self):
        newFromPoint = Point(self.toPoint[0], self.toPoint[1])
        newToPoint = Point(self.fromPoint[0], self.fromPoint[1])
        return Edge(newFromPoint, newToPoint)

    def doesIntersect(self, otherEdge):
        p = self.fromPoint
        r = self.toPoint - self.fromPoint
        q = otherEdge.fromPoint
        s = otherEdge.toPoint - otherEdge.fromPoint
        rCrossS = np.cross(r,s)
        qMinusPCrossR = np.cross((q-p),r)
        qMinusPCrossS = np.cross((q-p),s)

        #if either vertex in edge1 is in edge2 then we say this is not intersection but just meeting
        if np.array_equal(self.fromPoint, otherEdge.fromPoint) or np.array_equal(self.fromPoint, otherEdge.toPoint) \
            or np.array_equal(self.toPoint, otherEdge.fromPoint) or np.array_equal(self.toPoint, otherEdge.toPoint):
            return False

        # collinear
        if rCrossS == 0 and qMinusPCrossR == 0:
            if 0 <= np.dot((q-p),r) <= np.dot(r,r) or 0 <= np.dot((p-q),s) <= np.dot(s,s):
                # we are overlapping so intersection (but this still means you can squeeze past so allow) CHECK CHECK CHECK
                return False
            else:
                #disjoint
                return False
        if rCrossS == 0:
            # parallel but not intersecting
            return False
        # must not be parallel
        t = float(qMinusPCrossS) / rCrossS
        u = float(qMinusPCrossR) / rCrossS
        #there's an intersection but let's check that it's not at the end of either line segment
        if rCrossS != 0 and 0<= t <= 1 and 0 <= u <= 1:
            intersection = p + t *r
            if np.array_equal(intersection, otherEdge.fromPoint) or np.array_equal(intersection, otherEdge.toPoint) \
                or np.array_equal(intersection, self.fromPoint) or np.array_equal(intersection, self.toPoint):
                return False
            return True
        else:
            return False

class Polygon():
    #points are a list of tuples [point1, point2,...]
    def __init__(self, points):
        self.points = points
        self.edges = self.getEdges()

    def __str__(self):
        return str(self.points)
    def __repr__(self):
        return self.__str__()

    def translate(self, dx, dy):
        translatedPoints = [Point(point.x + dx, point.y +dy) for point in self.points]
        return Polygon(translatedPoints)

    def getEdges(self):
        edges = []
        # get edges in terms of tuples considering a counter-clockwise orientation
        numPoints = len(self.points)
        for i in range(numPoints):
            edges.append(Edge(self.points[i], self.points[(i+1) % numPoints]))
        return edges

    def rotate(self, theta, referencePoint):
        rotatedPoints = []
        cosTheta = math.cos(math.radians(theta))
        sinTheta = math.sin(math.radians(theta))
        rotationMatrix = np.array([[cosTheta, -sinTheta], [sinTheta, cosTheta]])
        for point in self.points:
            if point == referencePoint:
                rotatedPoints.append(point)
                continue
            else:
                vector = point.arr - referencePoint.arr
                rotatedPoint = rotationMatrix.dot(vector)
                newPoint = rotatedPoint + referencePoint.arr    
                rotatedPoints.append(Point(newPoint[0], newPoint[1]))
        return Polygon(rotatedPoints)

    def moveRelative(self, dx, dy):
        self.position =[polygon.translate(dx,dy) for polygon in self.position]

    # adapted from http://www.ariel.com.au/a/python-point-int-poly.html
    # by default we say points on the edge of the polygon are not considered inside
    def pointInPolygon(self, point, excludePointsOnBoundary):
        if excludePointsOnBoundary:
            #check if the point is on any edge of the polygon-- if so we say point is NOT in
            for edge in self.edges:
                if np.cross(edge.toPoint - edge.fromPoint, point.arr - edge.fromPoint) == 0:
                    # point is on an edge so this is not in the polygon
                    return False

        #not on edge so we can check normally
        numPoints = len(self.points)
        inside = False
        p1x,p1y = self.points[0].arr
        for i in range(numPoints+1):
            p2x,p2y = self.points[i % numPoints].arr
            if  min(p1y,p2y) < point.y <= max(p1y,p2y):
                    if point.x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (point.y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or point.x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside
    # assumes points defined ccw
    def getNormals(self, label):
        normals = []
        for edgeNumber, edge in enumerate(self.edges):
            fromPoint = edge.fromPoint
            toPoint = edge.toPoint
            dx = toPoint[0] - fromPoint[0]
            dy = toPoint[1] - fromPoint[1]
            if label == 'robot':
                #we want inside normals
                normals.append([-dy, dx, edgeNumber, label])
            else:
                # we want outside normals
                normals.append([dy, -dx, edgeNumber, label])

        return normals

    def doesOverlap(self, otherPolygon):
        # first check to see if any line segments intersect
        if self.doesIntersect(otherPolygon):
            return True
        # otherwise no edges intersect
        # now check to see if one object is fully contained in the other.
        # To check one object being on the border of the other we check all vertices until we see one contained
        return self.doesContainVertexOfPolygon(otherPolygon, True) or otherPolygon.doesContainVertexOfPolygon(self, True)
    
    def doesIntersect(self, otherPolygon):
        for otherEdge in otherPolygon.edges:
            if self.doesEdgeIntersect(otherEdge):
                return True
        return False

    def doesEdgeIntersect(self, otherEdge):
        for myEdge in self.edges:
            if myEdge.doesIntersect(otherEdge):
                return True
        return False

    def doesContainPolygon(self, otherPolygon):
        return (not self.doesIntersect(otherPolygon)) and self.doesContainVertexOfPolygon(otherPolygon, False)

    def doesContainVertexOfPolygon(self, otherPolygon, excludePointsOnBoundary = False):
        for otherPoint in otherPolygon.points:
            if self.pointInPolygon(otherPoint, excludePointsOnBoundary):
                return True
        return False
