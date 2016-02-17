from drawCommon import *
from testWorlds import *
import packagehelper

import math
from Tkinter import *
from mpl import algorithm_runner
from mpl.common.MPLHelper import MPLHelper
from mpl.common.world import World, SimpleObstacle
from mpl.common.movableLinkRobotWithObject import MovableLinkRobotWithObject

pi = math.pi
piOver2 = math.pi / 2
world, obstacles = get2DHandleAndCansWorld()
sim = makeSim(world)
links = []
links.append([0, [(205,225), (280,225), (280,275), (205,275)]])
links.append([0, [(280,225), (355,225), (355,275), (280,275)]])
links.append([0, [(355,225), (430,225), (430,275), (355,275)]])
heldObject = []
heldObject.append([(430, 150), (480, 150), (480, 350), (430, 350)])
start = (205,225, 0, 0, 0)
goal = (50, 50, 0, piOver2, -piOver2)
robot = MovableLinkRobotWithObject(links, heldObject, world)
helper = MPLHelper(robot, world, goal, 50)
drawProblemAndWait(sim, robot, obstacles, start, goal)
helper.collisionsAtQ(start)