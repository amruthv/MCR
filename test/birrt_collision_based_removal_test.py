from profiler import *
from drawCommon import *
from testWorlds import *
import packagehelper

import cProfile
import math
from mpl.algorithms.rrt_variants.birrt_collision_based_removal.birrt_collision_based_removal import BiRRTCollisionRemovalSearcher
from mpl.common import searcher
from mpl.common import movableLinkRobot
from mpl.common.world import World, SimpleObstacle
from mpl.common.simplemcrhelper import SimpleMCRHelper

pi = math.pi
piOver2 = math.pi / 2

def testOneObstacleMiddle():
    world, obstacles = getWorldMiddleObstacle()
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = movableLinkRobot.MovableLinkRobot(links, world)
    helper = SimpleMCRHelper(linkRobot, world, goal, 50)
    birrt = BiRRTCollisionRemovalSearcher(start, goal, helper, False)
    
    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    
    if birrt.run():
        path = birrt.getPath()
        cover = birrt.getCover()
        print 'path =', path
        print 'cover =', cover
        sim.drawPath(obstacles, linkRobot, path)

def testManyObstacles(verbose = False):
    world, obstacles = getManyObstaclesWeightedWorld

    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    start = (20,200, 0, 0, 0)
    goal = (450, 360, 0, piOver2, piOver2)
    linkRobot = movableLinkRobot.MovableLinkRobot(links, world)
    helper = SimpleMCRHelper(linkRobot, world, goal, 50)
    sim = makeSim(world)

    birrt = BiRRTCollisionRemovalSearcher(start, goal, helper, useTLPObstacles = False, removalStrategy = 'greedy')
    
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    
    if birrt.run():
        path = birrt.getPath()
        cover = birrt.getCover()
        print 'path =', path
        print 'cover =', cover
        sim.drawPath(obstacles, linkRobot, path)

if __name__ == '__main__':
    testOneObstacleMiddle()
    testManyObstacles()
    # prof('testManyObstacles()')

