from drawCommon import *
from testWorlds import *
import packagehelper

import math
import time
import multiprocessing
import cProfile
from Tkinter import *
from mpl.common.movableLinkRobot import MovableLinkRobot
from mpl.common.MPLHelper import MPLHelper
from mpl.algorithms.mcr.mcr import MCRPlanner

pi = math.pi
piOver2 = math.pi / 2

def testNoObstacles():
    world, obstacles = getEmptyWorld()
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (300, 300, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = MPLHelper(linkRobot, world, goal, 50)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot,obstacles, start, goal)

    mcr = MCRPlanner(start, goal, mcrhelper, False)
    mcr.run()

def testOneObstacleMiddle():
    world, obstacles = getWorldMiddleObstacle()

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)

    mcrhelper = MPLHelper(linkRobot, world, goal, 50)
    mcr = MCRPlanner(start, goal, mcrhelper, False)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    
    s_min = mcr.run()
    path = mcr.getPath()
    cover = mcr.getCover()
    print 'path:', path
    print 'cover:', cover
    sim.drawPath(obstacles, linkRobot, path)


def testTwoDiffWeightObstacles():
    world, obstacles = getTwoObstaclesDifferentWeightWorld()

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = MPLHelper(linkRobot, world, goal, 50)
    mcr = MCRPlanner(start, goal, mcrhelper, False)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    s_min = mcr.run()
    path = mcr.getPath()
    cover = mcr.getCover()
    print 'path:', path
    print 'cover:', cover
    sim.drawPath(obstacles, linkRobot, path)

def testManyObstacles():
    world, obstacles = getManyObstaclesWeightedWorld()

    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    start = (20,200, 0, 0, 0)
    goal = (450, 360, 0, piOver2, piOver2)
    linkRobot = MovableLinkRobot(links, world)
    mcrhelper = MPLHelper(linkRobot, world, goal, 50)
    mcr = MCRPlanner(start, goal, mcrhelper, useTLPObstacles = False, verbose = False)
    
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

# testNoObstacles()
# testOneObstacleMiddle()
# testTwoDiffWeightObstacles()
testManyObstacles()
# prof('testManyObstacles()')
# cProfile.run('testTwoDiffWeightObstacles()')

# print runNTimes(testManyObstacles, 5)
# print runNTimes(testManyObstacles2Links, 5)
