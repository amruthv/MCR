from mcr import MCRPlanner
from world import World, Obstacle
from movableLinkRobot import MovableLinkRobot
from Tkinter import *
from simulator import *
import searcher
import cProfile
import math
from simplemcrhelper import SimpleMCRHelper

pi = math.pi
piOver2 = math.pi / 2

def testNoObstacles():
    master = Tk()
    canvas = Canvas(master, width=500, height=500)
    canvas.pack()
    sim = Simulator(canvas, 500, 500)
    world = World(500,500, [])
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (300, 300, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = SimpleMCRHelper(linkRobot, world)
    mcr = MCRPlanner(start, goal, mcrhelper, sim)
    mcr.discreteMCR()

def testOneObstacleMiddle():
    master = Tk()
    canvas = Canvas(master, width=500, height=500)
    canvas.pack()
    sim = Simulator(canvas, 500, 500)

    obstacle1 = SimpleObstacle([(200,20), (300,20), (300,150), (200,150)])
    obstacles = [obstacle1]
    sim.drawObstacles(obstacles)
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    sim.drawRobot(linkRobot)
    linkRobot.moveToConfiguration(goal)
    sim.drawRobot(linkRobot)
    raw_input()

    mcrhelper = SimpleMCRHelper(linkRobot, world)
    mcr = MCRPlanner(start, goal, mcrhelper, sim)
    sim.drawPoint((start[0], start[1]), fill = 'green')
    sim.drawPoint((goal[0], goal[1]), fill = 'blue')
    mcr.discreteMCR()
    path = searcher.reconstructPath(mcr.cameFrom, goal)
    drawPath(sim, obstacles, linkRobot, path)


def testTwoDiffWeightObstacles():
    master = Tk()
    canvas = Canvas(master, width=500, height=500)
    canvas.pack()
    sim = Simulator(canvas, 500, 500)

    obstacle1 = SimpleObstacle([(200,20), (300,20), (300,150), (200,150)], 4)
    obstacle2 = SimpleObstacle([(200,150), (300,150), (300,500), (200,500)], 1)
    obstacles = [obstacle1, obstacle2]
    sim.drawObstacles(obstacles)
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = SimpleMCRHelper(linkRobot, world)
    mcr = MCRPlanner(start, goal, mcrhelper, sim)
    try:
        cameFrom = mcr.discreteMCR()
    except KeyboardInterrupt:
        drawGraph(sim, obstacles, mcr.G)
    # path = searcher.reconstructPath(cameFrom, goal)
    # drawPath(sim, obstacles, linkRobot, path)

def testManyObstacles():
    master = Tk()
    canvas = Canvas(master, width=500, height=500)
    canvas.pack()
    sim = Simulator(canvas, 500, 500)
    obstacle1 = SimpleObstacle([(70,0), (245,0), (270,100), (160,180), (45, 100)], 4)
    obstacle2 = SimpleObstacle([(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle([(280, 200), (350, 200), (350, 375), (280, 375)])
    obstacle4 = SimpleObstacle([(375, 20), (480, 20), (480, 110), (375, 110)])
    obstacle5 = SimpleObstacle([(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
    sim.drawObstacles(obstacles)
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    start = (20,200, 0, 0, 0)
    goal = (450, 360, 0, piOver2, piOver2)
    linkRobot = MovableLinkRobot(links, world)
    sim.drawRobot(linkRobot)
    linkRobot.moveToConfiguration(goal)
    sim.drawRobot(linkRobot)
    raw_input()
    mcrhelper = SimpleMCRHelper(linkRobot, world)
    mcr = MCRPlanner(start, goal, mcrhelper, sim)
    try:
        cameFrom = mcr.discreteMCR()
    except KeyboardInterrupt:
        drawGraph(sim, obstacles, mcr.G)

def testManyObstacles2Links():
    master = Tk()
    canvas = Canvas(master, width=500, height=500)
    canvas.pack()
    sim = Simulator(canvas, 500, 500)
    obstacle1 = SimpleObstacle([(100,0), (275,0), (300,100), (190,180), (75, 100)], 4)
    obstacle2 = SimpleObstacle([(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle([(250, 200), (350, 200), (350, 375), (250, 375)])
    obstacle4 = SimpleObstacle([(375, 50), (480, 50), (480, 140), (375, 140)])
    obstacle5 = SimpleObstacle([(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
    sim.drawObstacles(obstacles)
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    start = (20,200, 0, 0)
    goal = (450, 360, 0, piOver2)
    linkRobot = MovableLinkRobot(links, world)
    linkRobot.moveToConfiguration(start)
    sim.drawRobot(linkRobot)
    linkRobot.moveToConfiguration(goal)
    sim.drawRobot(linkRobot)
    mcrhelper = SimpleMCRHelper(linkRobot, world)
    mcr = MCRPlanner(start, goal, mcrhelper, sim)
    try:
        cameFrom = mcr.discreteMCR()
    except KeyboardInterrupt:
        raw_input()
        drawGraph(sim, obstacles, mcr.G)

def drawGraph(sim, obstacles, G):
    sim.clearCanvas()
    sim.drawObstacles(obstacles)
    for V in G.V:
        sim.drawPoint((V[0], V[1]))
    # for v1 in G.E:
    #     for v2 in G.E[v1]:
    #         sim.drawLine(v1[0], v1[1], v2[0], v2[1])
    raw_input()

def drawPath(sim, obstacles, robot, path):
    for q in path:
        sim.clearCanvas()
        sim.drawObstacles(obstacles)
        robot.moveToConfiguration(q)
        sim.drawRobot(robot)
        raw_input()

def prof(test, n=50):
    import cProfile
    import pstats
    try:
        cProfile.run(test, 'prof')
    except:
        print 'Done'
    p = pstats.Stats('prof')
    p.sort_stats('cumulative').print_stats(n)

testManyObstacles2Links()
# obstacle1 = Obstacle([(100,0), (275,0), (300,100), (190,180), (75, 100)], 4)
# obstacle2 = Obstacle([(140, 385), (225,425), (140,480), (55,425)], 1)
# obstacle3 = Obstacle([(250, 200), (350, 200), (350, 375), (250, 375)])
# obstacle4 = Obstacle([(375, 90), (480, 90), (480, 180), (375, 180)])
# obstacle5 = Obstacle([(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
# obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
# world = World(500,500, obstacles)


# testNoObstacles()
# testOneObstacleMiddle()
# testTwoDiffWeightObstacles()
# testManyObstacles()
# testManyObstacles2Links()
# prof('testManyObstacles()')
# cProfile.run('testTwoDiffWeightObstacles()')