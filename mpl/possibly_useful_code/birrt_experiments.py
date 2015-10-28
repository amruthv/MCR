from polygonhelper import *
from polygonalRobot import *
from linkRobot import *
from world import World
from simulator import *
from Tkinter import *
from birrt import *
import searcher
from collisionchecker import *
import smoother

#experiment with polygonal robots and biRRT
def experiment1(shouldDraw = True, shouldSmooth = True):
    master = Tk()
    canvas = Canvas(master, width=400, height=200)
    canvas.pack()

    robotPolygon = [Point(20, 80), Point(48,52), Point(76,80), Point(48,108)]
    obstacle1 = [Point(105,20), Point(185,20), Point(185,60), Point(105,60)]
    obstacle2 = [Point(100,110), Point(180,110), Point(180,180), Point(100,180)]
    obstaclesPoints = [obstacle1, obstacle2]
    obstacles = [Polygon(obstaclePoints) for obstaclePoints in obstaclesPoints]
    # obstacles = []
    start = [20, 80, 0]
    goal = [200,100, 0]
    world = World(400, 200)
    robot = PolygonalRobot([robotPolygon], world, goal)
    isParamDegree = [False, False, False]
    collisionChecker = CollisionChecker(robot, obstacles, isParamDegree)
    birrt = BiRRT(start, 0.1, goal, robot.generateRandomConfiguration, collisionChecker.validateTrajectory)
    meetingPoint = birrt.growUntilFound()
    if meetingPoint == goal:
        path = searcher.reconstructPath(birrt.startRRT.cameFrom, tuple(goal))
    else:
        firstPart = searcher.reconstructPath(birrt.startRRT.cameFrom, tuple(meetingPoint))
        secondPart = searcher.reconstructPath(birrt.goalRRT.cameFrom, tuple(meetingPoint))
        secondPart.reverse()
        path = firstPart + secondPart[1:]

    if path == None:
        print 'path was None'
        return None
    if shouldSmooth:
        smoothedPath = smoother.smoothPath(path, collisionChecker.validateTrajectory)
        print 'path length', len(path)
        print 'smoothedPath', len(smoothedPath)
        if shouldDraw:
            for step in smoothedPath:
                robot.moveToConfiguration(step)
                canvas.delete("all")
                drawRobot(robot, canvas)
                drawObstacles(obstacles, canvas)
                drawPoint(start, canvas, fill='purple')
                drawPoint(goal, canvas, fill='cyan')
                previous = step
                raw_input()
        return searcher.costOfPath(smoothedPath)
    else:
        return searcher.costOfPath(path)

#experiments with link robot and bi rrt
def experiment2(shouldDraw = True, shouldSmooth = True):
    master = Tk()
    canvas = Canvas(master, width=400, height=200)
    canvas.pack()

    obstacle1 = [Point(110,80), Point(160,80), Point(160,100), Point(100,100)]
    obstacle2 = [Point(10,65), Point(25, 65), Point(25,140), Point(10,140)]
    obstaclesPoints = [obstacle1, obstacle2]
    obstacles = [Polygon(obstaclePoints) for obstaclePoints in obstaclesPoints]
    # obstacles = []
    start = [0, 0, 0]
    goal = [0,90, -90]
    world = World(400, 200)
    links = []
    links.append([0, [(20,40), (60,40), (60,60), (20,60)]])
    links.append([0, [(60,40), (100,40), (100,60), (60,60)]])
    links.append([0, [(100,40), (140,40), (140,60), (100,60)]])
    linkRobot = LinkRobot(links, world, goal)
    isParamDegree = [False, False, False]
    collisionChecker = CollisionChecker(linkRobot, obstacles, isParamDegree)
    birrt = BiRRT(start, 0.1, goal, linkRobot.generateRandomConfiguration, collisionChecker.validateTrajectory)
    meetingPoint = birrt.growUntilFound()
    if meetingPoint == goal:
        path = searcher.reconstructPath(birrt.startRRT.cameFrom, tuple(goal))
    else:
        firstPart = searcher.reconstructPath(birrt.startRRT.cameFrom, tuple(meetingPoint))
        secondPart = searcher.reconstructPath(birrt.goalRRT.cameFrom, tuple(meetingPoint))
        print 'firstPart', firstPart
        print 'original second part', secondPart
        secondPart.reverse()
        print 'secondPart', secondPart
        path = firstPart + secondPart[1:]

    if path == None:
        print 'path was None'
        return None
    if shouldSmooth:
        smoothedPath = smoother.smoothPath(path, collisionChecker.validateTrajectory)
        print 'path length', len(path)
        print 'smoothedPath', len(smoothedPath)
        if shouldDraw:
            for step in smoothedPath:
                linkRobot.moveToConfiguration(step)
                canvas.delete("all")
                drawRobot(linkRobot, canvas)
                drawObstacles(obstacles, canvas)
                drawPoint(start, canvas, fill='purple')
                drawPoint(goal, canvas, fill='cyan')
                previous = step
                raw_input()
        return searcher.costOfPath(smoothedPath)
    else:
        return searcher.costOfPath(path)


#experiment with polygonal robots and biRRT but with a larger world for 2 options to go around obstacle
def experiment3(shouldDraw = True, shouldSmooth = True):
    master = Tk()
    canvas = Canvas(master, width=400, height=400)
    canvas.pack()

    robotPolygon = [Point(20, 80), Point(48,52), Point(76,80), Point(48,108)]
    obstacle1 = [Point(105,20), Point(185,20), Point(185,60), Point(105,60)]
    obstacle2 = [Point(100,110), Point(180,110), Point(180,180), Point(100,180)]
    obstaclesPoints = [obstacle1, obstacle2]
    obstacles = [Polygon(obstaclePoints) for obstaclePoints in obstaclesPoints]
    # obstacles = []
    start = [20, 80, 0]
    goal = [200,100, 0]
    world = World(400, 200)
    robot = PolygonalRobot([robotPolygon], world, goal)
    isParamDegree = [False, False, False]
    collisionChecker = CollisionChecker(robot, obstacles, isParamDegree)
    birrt = BiRRT(start, 0.1, goal, robot.generateRandomConfiguration, collisionChecker.validateTrajectory)
    meetingPoint = birrt.growUntilFound()
    if meetingPoint == goal:
        path = searcher.reconstructPath(birrt.startRRT.cameFrom, tuple(goal))
    else:
        firstPart = searcher.reconstructPath(birrt.startRRT.cameFrom, tuple(meetingPoint))
        secondPart = searcher.reconstructPath(birrt.goalRRT.cameFrom, tuple(meetingPoint))
        secondPart.reverse()
        path = firstPart + secondPart[1:]

    if path == None:
        print 'path was None'
        return None
    if shouldSmooth:
        smoothedPath = smoother.smoothPath(path, collisionChecker.validateTrajectory)
        print 'path length', len(path)
        print 'smoothedPath', len(smoothedPath)
        if shouldDraw:
            for step in smoothedPath:
                robot.moveToConfiguration(step)
                canvas.delete("all")
                drawRobot(robot, canvas)
                drawObstacles(obstacles, canvas)
                drawPoint(start, canvas, fill='purple')
                drawPoint(goal, canvas, fill='cyan')
                previous = step
                raw_input()
        return searcher.costOfPath(smoothedPath)
    else:
        return searcher.costOfPath(path)


# experiment1()
# experiment2()
# experiment3()