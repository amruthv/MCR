import math
import time
import pdb
import sys

from testWorlds import *
from drawCommon import makeSim, drawProblemAndWait, drawPath
import simulator
import packagehelper

from mpl.common.movableLinkRobotWithObject import MovableLinkRobotWithObject
from mpl.common.MPLHelper import MPLHelper
from mpl.common.cover import Cover
from mpl.algorithms.rrt_variants.birrt_collision_based_removal.birrt_collision_based_removal import BiRRTCollisionRemovalSearcher
from mpl.common.configuration import Configuration

draw = False
numTimesToRunEach = 20
stepSize = 1150
pi = math.pi
piOver2 = math.pi / 2
discountFactors = [0.1, 0.3, 0.5, 0.7, 0.9]


def runOnEmptyWorld():
    world, obstacles = getEmptyWorld()
    links = []
    links.append([0, [(205,225), (280,225), (280,275), (205,275)]])
    links.append([0, [(280,225), (355,225), (355,275), (280,275)]])
    links.append([0, [(355,225), (430,225), (430,275), (355,275)]])
    heldObject = []
    heldObject.append([(430, 150), (480, 150), (480, 350), (430, 350)])
    start = Configuration([205,225], [0, 0, 0])
    goal = Configuration([50, 50], [0, piOver2, -piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runWorldWithDiscountFactor(start, goal, helper, robot, world, discountFactors)   
    writeTestResults('GreedyEmptyWorld', testResults, robot)

def runOnSomeObstaclesFeasibleWorld():
    world, obstacles = getWorldWithFeasible()
    links = []
    links.append([0, [(205,225), (280,225), (280,275), (205,275)]])
    links.append([0, [(280,225), (355,225), (355,275), (280,275)]])
    links.append([0, [(355,225), (430,225), (430,275), (355,275)]])
    heldObject = []
    heldObject.append([(430, 150), (480, 150), (480, 350), (430, 350)])
    start = Configuration([205,225], [0, 0, 0])
    goal = Configuration([50, 50], [0, piOver2, -piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runWorldWithDiscountFactor(start, goal, helper, robot, world, discountFactors)   
    writeTestResults('GreedySomeObstaclesFeasibleWorld', testResults, robot)

def runOnTwoSoda():
    world, obstacles = get2DHandleAndCansWorld()
    links = []
    links.append([0, [(205,225), (280,225), (280,275), (205,275)]])
    links.append([0, [(280,225), (355,225), (355,275), (280,275)]])
    links.append([0, [(355,225), (430,225), (430,275), (355,275)]])
    heldObject = []
    heldObject.append([(430, 150), (480, 150), (480, 350), (430, 350)])
    start = Configuration([205,225], [0, 0, 0])
    goal = Configuration([50, 50], [0, piOver2, -piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runWorldWithDiscountFactor(start, goal, helper, robot, world, discountFactors)   
    writeTestResults('GreedyTwoSodaHandle', testResults, robot)

def runOnClutteredWorld():
    world, obstacles = get2DHandleAndClutteredWorld()
    links = []
    links.append([0, [(205,225), (280,225), (280,275), (205,275)]])
    links.append([0, [(280,225), (355,225), (355,275), (280,275)]])
    links.append([0, [(355,225), (430,225), (430,275), (355,275)]])
    heldObject = []
    heldObject.append([(430, 150), (480, 150), (480, 350), (430, 350)])
    start = Configuration([205,225], [0, 0, 0])
    goal = Configuration([70, 0], [piOver2, 0, -piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runWorldWithDiscountFactor(start, goal, helper, robot, world, discountFactors)   
    writeTestResults('GreedyClutteredWorld', testResults, robot)

def runOnTopLightClutteredWorld():
    world, obstacles = get2DHandleAndTopLightClutteredWorld()
    links = []
    links.append([0, [(205,225), (280,225), (280,275), (205,275)]])
    links.append([0, [(280,225), (355,225), (355,275), (280,275)]])
    links.append([0, [(355,225), (430,225), (430,275), (355,275)]])
    heldObject = []
    heldObject.append([(430, 150), (480, 150), (480, 350), (430, 350)])
    start = Configuration([205,225], [0, 0, 0])
    goal = Configuration([70, 0], [piOver2, 0, -piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runWorldWithDiscountFactor(start, goal, helper, robot, world, discountFactors)   
    writeTestResults('GreedyTopLightClutteredWorld', testResults, robot)

def runWorldWithDiscountFactor(start, goal, helper, robot, world, discountFactors):
    testResults = {}
    obstacles = world.obstacles
    for discountFactor in discountFactors:
        algorithmSuccessCount = 0.
        computationTime = 0.
        pathCost = 0.
        coverCost = 0.
        for i in range(numTimesToRunEach):
            t = time.time()
            searcher = BiRRTCollisionRemovalSearcher(start, goal, helper, False, 'greedy', discountFactor)
            searcher.run()
            path, cover = searcher.getPath(), searcher.getCover()
            if len(path) != 0:
                algorithmSuccessCount += 1
                pathCost += computeLengthOfPath(path, robot)
                coverOfPath = Cover(cover, False)
                coverCost += coverOfPath.score
                computationTime += time.time() - t
                print 'path =', path
                print 'cover = ', cover
                interpolatedPath = generateInterpolatedPath(path, helper)
                try:
                    assert(helper.nearEqual(interpolatedPath[0], start))
                    assert(helper.nearEqual(interpolatedPath[-1], goal))
                except:
                    pdb.set_trace()
                if draw:
                    drawPath(sim, obstacles, robot, interpolatedPath)
            else:
                print 'no path ;('
        if algorithmSuccessCount == 0:
            testResults[discountFactor] = (0, computationTime / numTimesToRunEach, 0, 0)
        else:
            testResults[discountFactor] = (100 * algorithmSuccessCount / numTimesToRunEach, computationTime / numTimesToRunEach, pathCost / algorithmSuccessCount, coverCost / algorithmSuccessCount)
    return testResults


def writeTestResults(testName, results, robot):
    import time
    currTime = time.strftime("%m-%d-%y_%H-%M")
    f = open('test_results/{0}_{1}.csv'.format(testName, currTime), 'w')
    f.write("discount factor, success rate, avg time, avg path length, avg cover score\n")
    for discountFactor in sorted(results.keys()):
        successFrequency, averageTime, averagePathLength, averagCoverScore = results[discountFactor]
        f.write("{0}, {1}, {2}, {3}, {4}\n".format(discountFactor, successFrequency, averageTime, averagePathLength, averagCoverScore))
    f.close()


def generateInterpolatedPath(path, helper):
    interpolatedPath = []
    for i in range(len(path) - 1):
        qs = [q for q in helper.generateInBetweenConfigs(path[i], path[i+1])]
        if i == 0:
            interpolatedPath += qs
        else:
            interpolatedPath += qs[1:]
    return interpolatedPath
    

def computeLengthOfPath(path, robot):
    dist = 0
    for i in range(len(path) - 1):
        dist += robot.distance(path[i], path[i+1])
    return dist

if __name__ == '__main__':
    if len(sys.argv) > 1:
        arg = int(sys.argv[1])
        if arg == 0:
            runOnEmptyWorld()
        elif arg == 1:
            runOnSomeObstaclesFeasibleWorld()
        elif arg == 2:
            runOnTwoSoda()
        elif arg == 3:
            runOnClutteredWorld()
        elif arg == 4:
            runOnTopLightClutteredWorld()
    else:
        runOnEmptyWorld()
        runOnSomeObstaclesFeasibleWorld()
        runOnTwoSoda()
        runOnClutteredWorld()
        runOnTopLightClutteredWorld()

