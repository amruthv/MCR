from profiler import *
from drawCommon import *
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
    obstacle1 = SimpleObstacle("obs1", [(200,20), (300,20), (300,150), (200,150)])
    obstacles = [] 
    obstacles = [obstacle1]
    world = World(500,350, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = movableLinkRobot.MovableLinkRobot(links, world)
    helper = SimpleMCRHelper(linkRobot, world, goal)
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
    obstacle1 = SimpleObstacle("obs1", [(100,0), (275,0), (300,100), (190,180), (75, 100)], 4)
    obstacle2 = SimpleObstacle("obs2", [(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle("obs3", [(250, 200), (350, 200), (350, 375), (250, 375)], 20)
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
    linkRobot = movableLinkRobot.MovableLinkRobot(links, world)
    helper = SimpleMCRHelper(linkRobot, world, goal)
    birrt = BiRRTCollisionRemovalSearcher(start, goal, helper, False)
    
    # sim = makeSim(world)
    # drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    
    if birrt.run():
        path = birrt.getPath()
        cover = birrt.getCover()
        # sim.drawPath(obstacles, linkRobot, path)
        # print cover

if __name__ == '__main__':
    # testOneObstacleMiddle()
    testManyObstacles()
    # prof('testManyObstacles()')

