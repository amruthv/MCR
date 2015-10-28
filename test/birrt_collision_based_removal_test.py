import packagehelper

import math
from Tkinter import *
from mpl.algorithms.rrt_variants.birrt_collision_based_removal.birrt_collision_based_removal import BiRRTCollisionRemovalSearcher
from mpl.common import searcher
from mpl.common import simulator
from mpl.common import movableLinkRobot
from mpl.common.world import World, SimpleObstacle
from mpl.common.simplemcrhelper import SimpleMCRHelper

pi = math.pi
piOver2 = math.pi / 2

def testOneObstacleMiddle():
    master = Tk()
    canvas = Canvas(master, width=500, height=350)
    canvas.pack()
    sim = simulator.Simulator(canvas, 500, 350)

    obstacle1 = SimpleObstacle([(200,20), (300,20), (300,150), (200,150)])
    obstacles = [] 
    obstacles = [obstacle1]
    sim.drawObstacles(obstacles)
    world = World(500,350, obstacles)

    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, 0, 0)
    goal = (320, 50, 0, piOver2, -piOver2)
    linkRobot = movableLinkRobot.MovableLinkRobot(links, world)
    sim.drawRobot(linkRobot)
    linkRobot.moveToConfiguration(goal)
    sim.drawRobot(linkRobot)
    raw_input()

    helper = SimpleMCRHelper(linkRobot, world, goal)
    birrt = BiRRTCollisionRemovalSearcher(start, goal, helper)
    sim.drawPoint((start[0], start[1]), fill = 'green')
    sim.drawPoint((goal[0], goal[1]), fill = 'blue')
    found = birrt.search()
    print 'found: ', found
    if found:
        path, cover = birrt.path()
        sim.drawPath(obstacles, linkRobot, path)

def testManyObstacles():
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
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    start = (20,200, 0, 0, 0)
    goal = (450, 360, 0, piOver2, piOver2)
    linkRobot = MovableLinkRobot(links, world)
    linkRobot.moveToConfiguration(start)
    sim.drawRobot(linkRobot)
    linkRobot.moveToConfiguration(goal)
    sim.drawRobot(linkRobot)
    raw_input()
    
    helper = SimpleMCRHelper(linkRobot, world, goal)
    birrt = BiRRTCollisionRemovalSearcher(start, goal, helper)
    sim.drawPoint((start[0], start[1]), fill = 'green')
    sim.drawPoint((goal[0], goal[1]), fill = 'blue')
    found = birrt.search()

    if found:
        path, cover = birrt.path()
        sim.drawPath(obstacles, linkRobot, path)
        print cover


if __name__ == '__main__':
    testOneObstacleMiddle()
