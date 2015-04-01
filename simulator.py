from polygonhelper import *
from DrawingWindowStandalone import *
from robot import *
from world import *
from Tkinter import *

class Simulator():
    def __init__(self, canvas, canvasWidth, canvasHeight):
        self.canvas = canvas
        self.width = canvasWidth
        self.height = canvasHeight
    def drawPoint(self, point, width = 5, fill = ''):
        x = point[0]
        y = point[1]
        self.canvas.create_oval([x-width, self.height - (y - width), x + width, self.height - (y + width)], fill = fill)
        self.canvas.update()

    def drawRobot(self, robot):
        for polygon in robot.position:
            self.drawPolygon(polygon, 'blue')
        self.canvas.update()

    def drawObstacles(self, obstacles):
        for obstacle in obstacles:
            self.drawPolygon(obstacle.polygon, 'red')
        self.canvas.update()

    def drawPolygon(self, polygon, color):
        points = []
        for point in polygon.points:
            points.append(point.x)
            points.append(self.height - point.y)
        self.canvas.create_polygon(points, fill=color, width=1, outline='black')
        self.canvas.update()

    def drawLine(self, x1, y1, x2, y2):
        self.canvas.create_line(x1, self.height - y1, x2, self.height - y2)
        self.canvas.update()

    def clearCanvas(self):
        self.canvas.delete("all")