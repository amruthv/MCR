# from polygonhelper import *
from DrawingWindowStandalone import *
from robot import *
from world import *
from Tkinter import *

class Simulator():
    def __init__(self, canvas, canvasWidth, canvasHeight):
        self.canvas = canvas
        self.width = canvasWidth
        self.height = canvasHeight
    def drawPoint(self, point, width = 5, fill = '', outline = 'black'):
        x = point[0]
        y = point[1]
        ptId = self.canvas.create_oval([x-width, self.height - (y - width), x + width, self.height - (y + width)], fill = fill, outline = outline)
        self.canvas.update()
        return ptId

    def drawRobot(self, robot):
        for polygon in robot.position:
            self.drawPolygon(polygon, 'blue')
        self.canvas.update()

    def drawObstacles(self, obstacles):
        for obstacle in obstacles:
            self.drawPolygon(obstacle.polygon, 'red')
            text_id = self.canvas.create_text(obstacle.polygon.centroid.x, self.height - obstacle.polygon.centroid.y)
            self.canvas.itemconfig(text_id, text=obstacle.weight)

        self.canvas.update()

    def drawPolygon(self, polygon, color):
        points = []
        for point in polygon.exterior.coords[:-1]:
            points.append(point[0])
            points.append(self.height - point[1])
        self.canvas.create_polygon(points, fill=color, width=1, outline='black')
        self.canvas.update()

    def drawLine(self, x1, y1, x2, y2, fill = 'black'):
        lineId = self.canvas.create_line(x1, self.height - y1, x2, self.height - y2, fill = fill)
        self.canvas.update()
        return lineId

    def clearCanvas(self):
        self.canvas.delete("all")

    def deleteObj(self, objId):
        self.canvas.delete(objId)