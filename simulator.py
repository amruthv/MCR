from polygonhelper import *
from DrawingWindowStandalone import *
from robot import *
from world import *
from Tkinter import *

class Simulator():
    def __init__(self, canvasWidth, canvasHeight):
        self.width = canvasWidth
        self.height = canvasHeight
    def drawPoint(self, point, canvas, width = 5, fill = ''):
        x = point[0]
        y = point[1]
        canvas.create_oval([x-width, self.height - (y - width), x + width, self.height - (y + width)], fill = fill)
        canvas.update()

    def drawRobot(self, robot, canvas):
        for polygon in robot.position:
            print polygon
            self.drawPolygon(polygon, canvas, 'blue')
        canvas.update()

    def drawObstacles(self, obstacles, canvas):
        for obstacle in obstacles:
            self.drawPolygon(obstacle, canvas, 'red')
        canvas.update()

    def drawPolygon(self, polygon, canvas, color):
        points = []
        for point in polygon.points:
            points.append(point.x)
            points.append(self.height - point.y)
        canvas.create_polygon(points, fill=color, width=1, outline='black')
        canvas.update()

    def drawAdjacency(self, adjacencyList, canvas):
        for point in adjacencyList:
            for otherPoint in adjacencyList[point]:
                canvas.create_line(point[0], point[1])
