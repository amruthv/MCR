from linkRobot import LinkRobot
from polygonalRobot import PolygonalRobot
from world import World
from polygonhelper import *
import pprint
from collisionchecker import *
from Tkinter import *
from simulator import *


def test1():
    links = []
    world = World(500,500)
    links.append([0, [(0,0), (4,0), (4,2), (0,2)]])
    links.append([0, [(4,0), (8,0), (8,2), (4,2)]])
    links.append([0, [(8,0), (12,0), (12,2), (8,2)]])
    linkRobot = LinkRobot(links, world, [0,0,0])
    print 'initial positions'
    pprint.pprint(linkRobot.position)
    linkRobot.moveToConfiguration([0,90,-90])
    print 'after rotations'
    pprint.pprint(linkRobot.position)
    linkRobot.moveToConfiguration([0,0,0])
    print 'testing rotation again back to original flat'
    pprint.pprint(linkRobot.position)

def test2():
    world = World(500,500)
    robotPolygons = []
    robotPolygons.append([Point(0,0), Point(4,0), Point(4,2), Point(0,2)])
    robotPolygons.append([Point(4,2), Point(8,2), Point(8,4), Point(4,4)])
    robot = PolygonalRobot(robotPolygons, world, [0,0,0])
    print 'initial'
    print robot.position
    robot.moveToConfiguration([2,0, 3])
    print 'rotated'
    print robot.position

    # master = Tk()
    # canvas.pack()

def test3():
    master = Tk()
    canvas = Canvas(master, width=400, height=200)
    canvas.pack()


    q_near = [201, 98, 357]
    q_near = [200, 103, 6]
    q_goal = [200,100, 0]

    robotPolygon = [Point(20, 80), Point(48,52), Point(76,80), Point(48,108)]
    obstacle1 = [Point(105,20), Point(185,20), Point(185,60), Point(105,60)]
    obstacle2 = [Point(100,110), Point(180,110), Point(180,180), Point(100,180)]
    obstaclesPoints = [obstacle1, obstacle2]
    obstacles = [Polygon(obstaclePoints) for obstaclePoints in obstaclesPoints]
    start = [20, 80, 0]
    goal = [200,100, 0]
    world = World(400, 200)
    robot = PolygonalRobot([robotPolygon], world, goal)
    isParamDegree = [False, False, True]
    collisionChecker = CollisionChecker(robot, obstacles, isParamDegree)
    configurations = collisionChecker.buildInBetweenConfigurations(q_near,q_goal)
    configurations.insert(0, q_near)
    print configurations
    for step in configurations:
        robot.moveToConfiguration(step)
        canvas.delete("all")
        drawRobot(robot, canvas)
        drawObstacles(obstacles, canvas)
        drawPoint(start, canvas, fill='purple')
        drawPoint(goal, canvas, fill='cyan')
        previous = step
        raw_input()
    pprint.pprint(configurations)

def test4():
    master = Tk()
    canvas = Canvas(master, width=400, height=200)
    canvas.pack()

    q_rand = [0, 90, -90]
    q_near = [0.0, 28.2842712474619, -28.2842712474619]
    print 'q_rand', q_rand
    print 'q_near', q_near

    obstacle1 = [Point(120,80), Point(160,80), Point(160,100), Point(120,100)]
    # obstacle2 = [Point(100,110), Point(180,110), Point(180,180), Point(100,180)]
    obstaclesPoints = [obstacle1]
    obstacles = [Polygon(obstaclePoints) for obstaclePoints in obstaclesPoints]
    # obstacles = []
    start = [0, 0, 0]
    goal = [0,90, -90]
    world = World(400, 200)
    links = []
    links.append([0, [(20,40), (60,40), (60,60), (20,60)]])
    links.append([0, [(60,40), (100,40), (100,60), (60,60)]])
    links.append([0, [(100,40), (140,40), (140,60), (100,60)]])
    linkRobot = LinkRobot(links, world, [0, 90, -90])
    isParamDegree = [False, False, False]
    collisionChecker = CollisionChecker(linkRobot, obstacles, isParamDegree)
    configurations = collisionChecker.buildInBetweenConfigurations(q_near,q_rand)
    configurations.insert(0, q_near)
    pprint.pprint(configurations)
    for step in configurations:
        linkRobot.moveToConfiguration(step)
        pprint.pprint('position' + str(linkRobot.position))
        canvas.delete("all")
        drawRobot(linkRobot, canvas)
        drawObstacles(obstacles, canvas)
        drawPoint(start, canvas, fill='purple')
        drawPoint(goal, canvas, fill='cyan')
        previous = step
        raw_input()
    # drawObstacles(obstacles, canvas)
    # drawRobot(linkRobot, canvas)
    # # drawPoint(goal, canvas)
    # raw_input()
    # linkRobot.moveToConfiguration([0,90,-90])
    # canvas.delete("all")
    # isParamDegree = [True, True, True]
    # drawObstacles(obstacles, canvas)    
    # drawRobot(linkRobot, canvas)
    # raw_input()

    # collisionChecker = CollisionChecker(linkRobot, obstacles, isParamDegree)
    # rrt = RRT(start, 0.1, goal, linkRobot.generateRandomConfiguration, collisionChecker.validateTrajectory)
    # rrt.growUntilFound()
    # cameFrom = rrt.cameFrom



# test1()
# test2()
# test3()
test4()
