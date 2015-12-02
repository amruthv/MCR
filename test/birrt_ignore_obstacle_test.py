from drawCommon import *
import packagehelper

import math
from mpl.common import searcher
from mpl.common.movableLinkRobot import MovableLinkRobot
from mpl.algorithms.rrt_variants.birrt_ignore_start_goal_obstacles.birrt_ignore_obstacles import BiRRTIgnoreObstacleSearcher
from mpl.common.world import World, Obstacle, SimpleObstacle
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
    linkRobot = MovableLinkRobot(links, world)

    helper = SimpleMCRHelper(linkRobot, world, goal)
    birrt = BiRRTIgnoreObstacleSearcher(start, goal, helper)

    sim = makeSim(world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
    
    if birrt.run():
        path = birrt.getPath()
        sim.drawPath(obstacles, linkRobot, path)


if __name__ == '__main__':
    testOneObstacleMiddle()