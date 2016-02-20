import math
import pdb

from testWorlds import *
from drawCommon import makeSim, drawProblemAndWait, drawPath
import simulator
import packagehelper

from mpl.common.movableLinkRobotWithObject import MovableLinkRobotWithObject
from mpl.common.MPLHelper import MPLHelper
from mpl.common.cover import Cover
import mpl.algorithm_runner as runner
from mpl.common.configuration import Configuration


algorithmNumberToStrategyMap = {0: 'MCR', 1: 'RRT', 2: 'BiRRT', 3: 'BiRRT ignore start and goal', 4: 'collision removal greedy', 5: 'collision removal probabilistic',
                                    6: 'ignore all non-permanent non-self', 7: 'collision removal repeat not greedy'}

draw = False
numTimesToRunEach = 5
def runAllOnTwoSoda():
    pi = math.pi
    piOver2 = math.pi / 2
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
    helper = MPLHelper(robot, world, goal, 75)
    testResults = {}
    if draw:
        sim = makeSim(world)
        drawProblemAndWait(sim, robot, obstacles, start, goal)
    for algorithmNumber in range(8):
        algorithmSuccessCount = 0.
        pathCost = 0.
        coverCost = 0.
        for i in range(numTimesToRunEach):
            path, cover = runner.runAlgorithm(start, goal, helper, algorithmNumber, False)
            if len(path) != 0:
                algorithmSuccessCount += 1
                pathCost += computeLengthOfPath(path, robot)
                coverCost += Cover(cover, False).score
                print 'path =', path
                print 'cover = ', cover
                if coverCost == 0:
                    print 'SOMETHING IS REALLY WRONG'
                    pdb.set_trace()
                if draw:
                    interpolatedPath = generateInterpolatedPath(path, helper)
                    drawPath(sim, obstacles, robot, interpolatedPath)
            else:
                print 'no path ;('
        if algorithmSuccessCount == 0:
            testResults[algorithmNumber] = (0, 0, 0)
        else:
            testResults[algorithmNumber] = (algorithmSuccessCount / numTimesToRunEach, pathCost / algorithmSuccessCount, coverCost / algorithmSuccessCount)

    writeTestResults('TwoSodaHandle', testResults, robot)


def writeTestResults(testName, results, robot):
    import time
    currTime = time.strftime("%m-%d-%y_%H:%M")
    f = open('test_results/{0}_{1}.csv'.format(testName, currTime), 'w')
    f.write("algorithm, path length, cover score\n")
    for algorithmNumber in results:
        algorithm = algorithmNumberToStrategyMap[algorithmNumber]
        successFrequency, averagePathLength, averagCoverScore = results[algorithmNumber]
        f.write("{0}, {1}, {2}, {3}\n".format(algorithm, successFrequency, averagePathLength, averagCoverScore))
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


runAllOnTwoSoda()