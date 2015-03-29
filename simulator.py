from polygonhelper import *
from DrawingWindowStandalone import *
from robot import *
from world import *
from Tkinter import *

def drawPoint(point, canvas, width = 5, fill = ''):
    x = point[0]
    y = point[1]
    canvas.create_oval([x-width, y - width, x + width, y + width], fill = fill)
    canvas.update()

def drawRobot(robot, canvas):
    for polygon in robot.position:
        drawPolygon(polygon, canvas, 'blue')
    canvas.update()

def drawObstacles(obstacles, canvas):
    for obstacle in obstacles:
        drawPolygon(obstacle, canvas, 'red')
    canvas.update()

def drawPolygon(polygon, canvas, color):
    points = []
    for point in polygon.points:
        points.append(point.x)
        points.append(point.y)
    canvas.create_polygon(points, fill=color, width=1, outline='black')
    canvas.update()

def drawAdjacency(adjacencyList, canvas):
    for point in adjacencyList:
        for otherPoint in adjacencyList[point]:
            canvas.create_line(point[0], point[1])
