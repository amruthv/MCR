import numpy as np
import random
import math
import pdb

from mcrhelper import MCRHelper
from shapely.geometry import Polygon
from bbox import BBox
from configuration import Configuration

class MPLHelper(MCRHelper):
    def __init__(self, robot, world, goal, stepSize):
        self.robot = robot
        self.world = world
        self.goal = goal
        self.stepSize = stepSize

    def collisionsAtQ(self, q):
        return self.robot.collisionsAtQ(q)

    def sampleConfig(self, goal):
        if random.random() < 0.1:
            return goal
        return self.robot.generateRandomConfiguration()

    def nearEqual(self, q1, q2):
        return all(x-y < 1.0e-6 for x,y in zip(q1.cartesianParameters, q2.cartesianParameters)) \
            and all(x-y < 1.0e-6 for x,y in zip(q1.angleParameters, q2.angleParameters))

    # don't need to worry about extendBackwards in rrt searches since this implementation
    # makes the same configurations when rotating either direction for better or worse
    def generateInBetweenConfigs(self, q_from, q_to):
        return Configuration.generateInBetweenConfigurations(q_from, q_to)

    def distance(self, q1, q2):
        return self.robot.distance(q1, q2)

    def getStepSize(self):
        return self.stepSize

    # THIS ONLY WORKS WITH L2 NORM ON DISTANCE FUNCTION FOR ROBOT
    def stepTowards(self, qFrom, qTo, stepSize = None):
        if stepSize is None:
            stepSize = self.stepSize
        dist = self.distance(qFrom, qTo)
        if dist < stepSize:
            return qTo
        bisectionCount = 0
        while True:
            bisectionCount += 1
            npQFromCartesian = np.array(qFrom.cartesianParameters)
            npQToCartesian = np.array(qTo.cartesianParameters)
            npQFromAngle = np.array(qFrom.angleParameters)
            npQToAngle = np.array(qTo.angleParameters)
            qPrimeCartesian = npQFromCartesian + 0.5 * (npQToCartesian - npQFromCartesian)
            qPrimeAngle = npQFromAngle + 0.5 * (npQToAngle- npQFromAngle)
            qPrime = Configuration(list(qPrimeCartesian), list(qPrimeAngle))
            if self.distance(qFrom, qPrime) < stepSize:
                # print 'bisected {0} times'.format(bisectionCount)
                return qPrime
            qTo = qPrime