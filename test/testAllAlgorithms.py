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
import mpl.algorithm_runner as runner
from mpl.common.configuration import Configuration


algorithmNumberToStrategyMap = {0: 'MCR', 1: 'RRT', 2: 'BiRRT', 3: 'TLP MCR', 4: 'Greedy Removal', 5: 'Probabilistic Removal',
                                    6: 'Ignore Non-Permanent', 7: 'Repeat Probabilistic Removal',
                                    8: 'Search Then Greedy Removal', 9: 'HPN Like Removal'}

draw = False
numTimesToRunEach = 200
stepSize = 1150
pi = math.pi
piOver2 = math.pi / 2


def runAllOnEmptyWorld():
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
    testResults = runAlgorithms(start, goal, helper, robot, world)   
    writeTestResults('EmptyWorld', testResults, robot)

def runAllOnSomeObstaclesFeasibleWorld():
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
    testResults = runAlgorithms(start, goal, helper, robot, world)    
    writeTestResults('SomeObstaclesFeasibleWorld', testResults, robot)

def runAllOnManyObstaclesWorld():
    world, obstacles = getManyObstaclesWorld()
    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    heldObject = []
    start = Configuration([20,200], [0, 0, 0])
    goal = Configuration([450, 360], [0, piOver2, piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runAlgorithms(start, goal, helper, robot, world)    
    writeTestResults('ManyObstaclesWorld', testResults, robot)

def runAllOnTwoSoda():
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
    testResults = runAlgorithms(start, goal, helper, robot, world)    
    writeTestResults('TwoSodaHandle', testResults, robot)

def runAllOnManyObstaclesUnfeasibleWorld():
    world, obstacles = getManyObstaclesUnfeasibleWorld()
    links = []
    links.append([0, [(20,200), (60,200), (60,220), (20,220)]])
    links.append([0, [(60,200), (100,200), (100,220), (60,220)]])
    links.append([0, [(100,200), (140,200), (140,220), (100,220)]])
    heldObject = []
    start = Configuration([20,200], [0, 0, 0])
    goal = Configuration([450, 360], [0, piOver2, piOver2])
    robot = MovableLinkRobotWithObject(links, heldObject, world)
    helper = MPLHelper(robot, world, goal, stepSize)
    testResults = runAlgorithms(start, goal, helper, robot, world)    
    writeTestResults('ManyObstaclesUnfeasibleWorld', testResults, robot)

def runAllOnClutteredWorld():
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
    testResults = runAlgorithms(start, goal, helper, robot, world)    
    writeTestResults('ClutteredWorld', testResults, robot)

def runAllOnTopLightClutteredWorld():
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
    testResults = runAlgorithms(start, goal, helper, robot, world)    
    writeTestResults('TopLightClutteredWorld', testResults, robot)

def runAlgorithms(start, goal, helper, robot, world):
    testResults = {}
    obstacles = world.obstacles
    if draw:
        sim = makeSim(world)
        drawProblemAndWait(sim, robot, obstacles, start, goal)
    for algorithmNumber in range(10):
        successTime = 0.
        successCount = 0.
        failureTime = 0.
        failureCount = 0.
        pathCost = 0.
        coverCost = 0.
        for i in range(numTimesToRunEach):
            t = time.time()
            path, cover = runner.runAlgorithm(start, goal, helper, algorithmNumber, False)
            if len(path) != 0:
                pathCost += computeLengthOfPath(path, robot)
                coverOfPath = Cover(cover, False)
                coverCost += coverOfPath.score
                successTime += time.time() - t
                successCount += 1
                print 'path =', path
                print 'cover = ', cover
                interpolatedPath = generateInterpolatedPath(path, helper)
                if draw:
                    drawPath(sim, obstacles, robot, path)
            else:
                print 'no path ;('
                failureTime += time.time() - t
                failureCount += 1
        if successCount == 0:
            testResults[algorithmNumber] = (0, failureTime / failureCount, 0, 0, 0)
        elif failureCount == 0:
            testResults[algorithmNumber] = (100, 0, successTime / successCount, pathCost / successCount, coverCost / successCount)
        else:
            testResults[algorithmNumber] = (100 * successCount / numTimesToRunEach, failureTime / failureCount, successTime / successCount,
                                            pathCost / successCount, coverCost / successCount)
    return testResults


def writeTestResults(testName, results, robot):
    import time
    currTime = time.strftime("%m-%d-%y_%H-%M")
    f = open('test_results/{0}_{1}.csv'.format(testName, currTime), 'w')
    f.write("algorithm, success rate, avg fail time, avg success time, avg path length, avg cover score\n")
    for algorithmNumber in sorted(results.keys()):
        algorithm = algorithmNumberToStrategyMap[algorithmNumber]
        successFrequency, averageTime, averagePathLength, averagCoverScore = results[algorithmNumber]
        f.write("{0}, {1}, {2}, {3}, {4}\n".format(algorithm, successFrequency, averageTime, averagePathLength, averagCoverScore))
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



# world, obstacles = getTest()
# sim = makeSim(world)
# sim.drawObstacles(obstacles)


# runAllOnEmptyWorld()
# runAllOnSomeObstaclesFeasibleWorld()
# runAllOnTwoSoda()
# runAllOnClutteredWorld()
# runAllOnTopLightClutteredWorld()
# runAllOnManyObstaclesWorld()
# runAllOnManyObstaclesUnfeasibleWorld()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        arg = int(sys.argv[1])
        if arg == 0:
            runAllOnEmptyWorld()
        elif arg == 1:
            runAllOnSomeObstaclesFeasibleWorld()
        elif arg == 2:
            runAllOnTwoSoda()
        elif arg == 3:
            runAllOnClutteredWorld()
        elif arg == 4:
            runAllOnTopLightClutteredWorld()
        elif arg == 5:
            runAllOnManyObstaclesWorld()
        elif arg == 6:
            runAllOnManyObstaclesUnfeasibleWorld()
    else:
        runAllOnEmptyWorld()
        runAllOnSomeObstaclesFeasibleWorld()
        runAllOnTwoSoda()
        runAllOnClutteredWorld()
        runAllOnTopLightClutteredWorld()
        runAllOnManyObstaclesWorld()
        runAllOnManyObstaclesUnfeasibleWorld()




