from drawCommon import *
import packagehelper

import math
import time
import multiprocessing
import cProfile
from Tkinter import *
from mpl.common.world import World, SimpleObstacle
from mpl.common.movableLinkRobot import MovableLinkRobot
from mpl.common import searcher
from mpl.common.simplemcrhelper import SimpleMCRHelper
from mpl.algorithms.mcr.mcr import MCRPlanner

pi = math.pi
piOver2 = math.pi / 2

def testNoObstacles():
    world = World(500,500, [])
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (300, 300, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = SimpleMCRHelper(linkRobot, world, goal)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, [], start, goal)

    mcr = MCRPlanner(start, goal, mcrhelper)
    mcr.run()

def testOneObstacleMiddle():
    obstacle1 = SimpleObstacle("obs1", [(200,20), (300,20), (300,150), (200,150)])
    obstacles = [obstacle1]
    world = World(500,350, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)

    mcrhelper = SimpleMCRHelper(linkRobot, world, goal)
    mcr = MCRPlanner(start, goal, mcrhelper)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    
    s_min = mcr.run()
    path = mcr.getPath()
    cover = mcr.getCover()
    print 'path:', path
    print 'cover:', cover
    sim.drawPath(obstacles, linkRobot, path)


def testTwoDiffWeightObstacles():
    obstacle1 = SimpleObstacle("obs1", [(200,20), (300,20), (300,150), (200,150)], 4)
    obstacle2 = SimpleObstacle("obs2", [(200,150), (300,150), (300,500), (200,500)], 1)
    obstacles = [obstacle1, obstacle2]
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = SimpleMCRHelper(linkRobot, world, goal)
    mcr = MCRPlanner(start, goal, mcrhelper)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    s_min = mcr.run()
    path = mcr.getPath()
    cover = mcr.getCover()
    print 'path:', path
    print 'cover:', cover
    sim.drawPath(obstacles, linkRobot, path)

def testManyObstacles():
    obstacle1 = SimpleObstacle("obs1", [(70,0), (245,0), (270,100), (160,180), (45, 100)], 4)
    obstacle2 = SimpleObstacle("obs2", [(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle("obs3", [(280, 200), (350, 200), (350, 375), (280, 375)])
    obstacle4 = SimpleObstacle("obs4", [(375, 20), (480, 20), (480, 110), (375, 110)])
    obstacle5 = SimpleObstacle("obs5", [(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    start = (20,200, 0, 0, 0)
    goal = (450, 360, 0, piOver2, piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = SimpleMCRHelper(linkRobot, world, goal)
    mcr = MCRPlanner(start, goal, mcrhelper)
    
    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    s_min = mcr.run()
    path = mcr.getPath()
    cover = mcr.getCover()
    print 'path:', path
    print 'cover:', cover
    sim.drawPath(obstacles, linkRobot, path)

def testManyObstacles2Links():
    obstacle1 = SimpleObstacle("obs1", [(100,0), (275,0), (300,100), (190,180), (75, 100)], 4)
    obstacle2 = SimpleObstacle("obs2", [(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle("obs3", [(250, 200), (350, 200), (350, 375), (250, 375)])
    obstacle4 = SimpleObstacle("obs4", [(375, 50), (480, 50), (480, 140), (375, 140)])
    obstacle5 = SimpleObstacle("obs5", [(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
    world = World(500,500, obstacles)

    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    start = (20,200, 0, 0, 0)
    goal = (450, 360, 0, piOver2, piOver2)
    linkRobot = MovableLinkRobot(links, world)
    
    mcrhelper = SimpleMCRHelper(linkRobot, world, goal)
    mcr = MCRPlanner(start, goal, mcrhelper, 20, False)
    
    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    s_min = mcr.run()
    path = mcr.getPath()
    cover = mcr.getCover()
    print 'path:', path
    print 'cover:', cover
    sim.drawPath(obstacles, linkRobot, path)

def drawGraph(sim, obstacles, G):
    sim.clearCanvas()
    sim.drawObstacles(obstacles)
    for V in G.V:
        sim.drawPoint((V[0], V[1]))
    # for v1 in G.E:
    #     for v2 in G.E[v1]:
    #         sim.drawLine(v1[0], v1[1], v2[0], v2[1])
    raw_input()

testNoObstacles()
testOneObstacleMiddle()
testTwoDiffWeightObstacles()
testManyObstacles()
testManyObstacles2Links()
# prof('testManyObstacles()')
# cProfile.run('testTwoDiffWeightObstacles()')

# print runNTimes(testManyObstacles, 5)
# print runNTimes(testManyObstacles2Links, 5)
