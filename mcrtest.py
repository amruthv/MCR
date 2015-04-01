from mcr import MCRPlanner
from world import World, Obstacle
from movableLinkRobot import MovableLinkRobot
from Tkinter import *
from simulator import *
import searcher
import cProfile

def testNoObstacles():
    # master = Tk()
    # canvas = Canvas(master, width=500, height=500)
    # canvas.pack()
    # sim = Simulator(canvas, 500, 500)
    world = World(500,500, [])
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (300, 300, 0, 90, -90)
    linkRobot = MovableLinkRobot(links, world)
    mcr = MCRPlanner(linkRobot, world, start, goal)
    mcr.discreteMCR()

def testOneObstacleMiddle():
    master = Tk()
    canvas = Canvas(master, width=500, height=500)
    canvas.pack()
    sim = Simulator(canvas, 500, 500)

    obstacle1 = Obstacle([Point(200,20), Point(300,20), Point(300,150), Point(200,150)])
    obstacles = [obstacle1]
    sim.drawObstacles(obstacles)
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, 90, -90)
    linkRobot = MovableLinkRobot(links, world)
    sim.drawRobot(linkRobot)
    linkRobot.moveToConfiguration(goal)
    sim.drawRobot(linkRobot)
    raw_input()

    mcr = MCRPlanner(linkRobot, world, start, goal)
    cameFrom = mcr.discreteMCR()
    path = searcher.reconstructPath(cameFrom, goal)
    drawPath(sim, obstacles, linkRobot, path)


def testTwoDiffWeightObstacles():
    # master = Tk()
    # canvas = Canvas(master, width=500, height=500)
    # canvas.pack()
    # sim = Simulator(canvas, 500, 500)

    obstacle1 = Obstacle([Point(200,20), Point(300,20), Point(300,150), Point(200,150)], 4)
    obstacle2 = Obstacle([Point(200,150), Point(300,150), Point(300,500), Point(200,500)], 1)
    obstacles = [obstacle1, obstacle2]
    # sim.drawObstacles(obstacles)
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, 90, -90)
    linkRobot = MovableLinkRobot(links, world)
    # sim.drawRobot(linkRobot)
    # linkRobot.moveToConfiguration(goal)
    # sim.drawRobot(linkRobot)
    # raw_input()
    mcr = MCRPlanner(linkRobot, world, start, goal)
    # try:
    mcr.discreteMCR()
        # cameFrom = 
    # except KeyboardInterrupt:
    #     drawGraph(sim, obstacles, mcr.G)
    # path = searcher.reconstructPath(cameFrom, goal)
    # drawPath(sim, obstacles, linkRobot, path)

def drawGraph(sim, obstacles, G):
    sim.clearCanvas()
    sim.drawObstacles(obstacles)
    for V in G.V:
        sim.drawPoint((V[0], V[1]))
    for v1 in G.E:
        for v2 in G.E[v1]:
            sim.drawLine(v1[0], v1[1], v2[0], v2[1])
    raw_input()

def drawPath(sim, obstacles, robot, path):
    for q in path:
        sim.clearCanvas()
        sim.drawObstacles(obstacles)
        robot.moveToConfiguration(q)
        sim.drawRobot(robot)
        raw_input()


# testNoObstacles()
# testOneObstacleMiddle()
cProfile.run('testTwoDiffWeightObstacles()')