import pdb
import packagehelper
from mpl.common.movableLinkRobot import MovableLinkRobot

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

    def changePointFill(self, objId, fill):
        self.canvas.itemconfig(objId, fill = fill)
        self.canvas.update()

    def drawConfiguration(self, robot, q, color):
        robot.moveToConfiguration(q)
        if isinstance(robot, MovableLinkRobot):
            self.drawRobotNoHeldObstacle(robot, color)
        else:
            self.drawRobotWithHeldObstacle(robot, color)


    def drawRobot(self, robot):
        if isinstance(robot, MovableLinkRobot):
            self.drawRobotNoHeldObstacle(robot)
        else:
            self.drawRobotWithHeldObstacle(robot)

    def drawRobotNoHeldObstacle(self, robot, robotColor = 'blue'):
        ids = []
        for polygon in robot.position:
            polyId = self.drawPolygon(polygon, robotColor)
            ids.append(polyId)
        self.canvas.update()
        return ids

    def drawRobotWithHeldObstacle(self, robot, robotColor = 'blue'):
        ids = []
        for polygon in robot.robotPosition:
            polyId = self.drawPolygon(polygon, robotColor)
            ids.append(polyId)
        for polygon in robot.heldObjectPosition:
            polyId = self.drawPolygon(polygon, 'green')
            ids.append(polyId)
        self.canvas.update()
        return ids

    def drawObstacles(self, obstacles):
        for obstacle in obstacles:
            self.drawPolygon(obstacle.points, 'red')
            text_id = self.canvas.create_text(obstacle.polygon.centroid.x, self.height - obstacle.polygon.centroid.y)
            self.canvas.itemconfig(text_id, text=str(obstacle))

        self.canvas.update()

    def drawPolygon(self, polygon, color):
        points = []
        for point in polygon:
            points.append(point[0])
            points.append(self.height - point[1])
        polyId = self.canvas.create_polygon(points, fill=color, width=1, outline='black')
        self.canvas.update()
        return polyId

    def drawLine(self, x1, y1, x2, y2, fill = 'black'):
        lineId = self.canvas.create_line(x1, self.height - y1, x2, self.height - y2, fill = fill)
        self.canvas.update()
        return lineId

    def clearCanvas(self):
        self.canvas.delete("all")

    def deleteManyObj(self, listOfObj):
        for obj in listOfObj:
            self.deleteObj(obj)

    def deleteObj(self, objId):
        self.canvas.delete(objId)

    def drawPath(self, obstacles, robot, path):
        for q in path:
            self.clearCanvas()
            self.drawObstacles(obstacles)
            robot.moveToConfiguration(q)
            self.drawRobot(robot)
            raw_input()