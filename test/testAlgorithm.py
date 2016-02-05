from drawCommon import *
from testWorlds import *
import packagehelper

from mpl.common.movableLinkRobotWithObject import MovableLinkRobotWithObject
from mpl.common.movableLinkRobot import MovableLinkRobot

from mpl import algorithm_runner
from mpl.common.simplemcrhelper import SimpleMCRHelper
import math

pi = math.pi
piOver2 = math.pi / 2

world, obstacles = getManySmallObstaclesWorld()
sim = makeSim(world)
# links = []
# links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
# links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
# links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
# heldObject = []
# heldObject.append([(170,50), (250,50), (250, 100), (170, 100)])
# heldObject.append([(250,50), (350,50), (350, 100), (250, 50)])
start = (50,50, 0, 0, 0)
goal = (300, 300, 0, piOver2, -piOver2)
# linkRobot = MovableLinkRobot(links, world)
# drawProblemAndWait(sim, linkRobot, obstacles, start, goal)
# linkRobotWithObject = MovableLinkRobotWithObject(links, heldObject, world)
# drawProblemAndWait(sim, linkRobotWithObject, obstacles, start, goal)
# helper = SimpleMCRHelper(linkRobot, world, goal, 50)

# path, cover = algorithm_runner.runAlgorithm(start, goal, helper, 5, useTLPObstacles = False)

# print path
# print cover


def linkRobot():
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    start = (50,50, 0, -piOver2, -piOver2)
    goal = (300, 300, 0, 0, 0)
    linkRobot = MovableLinkRobot(links, world)
    drawProblemAndWait(sim, linkRobot, obstacles, start, goal)

def holdingObstacleLinkRobot():
    links = []
    links.append([0, [(50,50), (90,50), (90,70), (50,70)]])
    links.append([0, [(90,50), (130,50), (130,70), (90,70)]])
    links.append([0, [(130,50), (170,50), (170,70), (130,70)]])
    heldObject = []
    heldObject.append([(170,50), (250,50), (250, 100), (170, 100)])
    # heldObject.append([(250,50), (350,50), (350, 100), (250, 100)])
    start = (50,50, 0, 0, 0)
    goal = (300, 300, 0, piOver2, 0)
    linkRobotWithObject = MovableLinkRobotWithObject(links, heldObject, world)
    drawProblemAndWait(sim, linkRobotWithObject, obstacles, start, goal)

# linkRobot()
holdingObstacleLinkRobot()