from drawCommon import *
from testWorlds import *
import packagehelper

import math
from Tkinter import *
from mpl import algorithm_runner
from mpl.common.simplemcrhelper import SimpleMCRHelper
from mpl.common.world import World, SimpleObstacle
from mpl.common.movableLinkRobot import MovableLinkRobot

pi = math.pi
piOver2 = math.pi / 2

world, obstacles = getManySmallObstaclesWorld()
links = []
links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
start = (50,50, 0, 0, 0)
goal = (300, 300, 0, piOver2, -piOver2)
linkRobot = MovableLinkRobot(links, world)
mcrhelper = SimpleMCRHelper(linkRobot, world, goal, 50)

sim = makeSim(world)
drawProblemAndWait(sim, linkRobot,obstacles, start, goal)