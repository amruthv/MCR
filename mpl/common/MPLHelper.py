import numpy as np
import random
import math

from mcrhelper import MCRHelper
from shapely.geometry import Polygon
from bbox import BBox
from configuration import Configuration

class MPLHelper(MCRHelper):
    def __init__(self, robot, world, goal, stepSize):
        self.robot = robot
        self.world = world
        self.goal = goal
        self.useBBoxChecker = True
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

    #inputs are -pi to pi
    def angleGenerator(self, q_from, q_to, stepsToCheck):
        # different sign
        if q_from * q_to < 0:
            if q_from < 0:
                negativeAngle = q_from
                positiveAngle = q_to
            else:
                negativeAngle = q_to
                positiveAngle = q_from
            twoPiShiftedNegativeAngle = negativeAngle + 2 * math.pi
            assert(positiveAngle - negativeAngle > 0)
            assert(twoPiShiftedNegativeAngle - positiveAngle > 0)
            if (negativeAngle - positiveAngle) > (twoPiShiftedNegativeAngle - positiveAngle):
                # we should use the addition of 2pi
                if positiveAngle == q_from:
                    qFrom = positiveAngle
                    qTo = twoPiShiftedNegativeAngle
                else:
                    qFrom = twoPiShiftedNegativeAngle
                    qTo = positiveAngle
                return normalInterpolateGenerator(qFrom, qTo, True)
        return normalInterpolateGenerator(q_from, q_to, False)

    # assumes that q_from and q_to are singular scalars
    def normalInterpolateGenerator(self, q_from, q_to, needsAngleAdjustment = False):
        q_from_np = np.array([q_from])
        q_to_np = np.array([q_to])
        q_delta = q_to_np - q_from_np
        for i in range(0, stepsToCheck + 1):
            q = q_from_np + q_delta * float(i) / stepsToCheck
            if needsAngleAdjustment and q > math.pi:
                yield q - 2 * math.pi
            yield q


    def distance(self, q1, q2):
        return self.robot.distance(q1, q2)

    def getStepSize(self):
        return self.stepSize

    # THIS ONLY WORKS WITH L2 NORM ON DISTANCE FUNCTION FOR ROBOT
    def stepTowards(self, qFrom, qTo, stepSize = None):
        if stepSize is None:
            stepSize = self.stepSize
        dist = self.distance(qFrom, qTo)
        if dist < self.stepSize:
            return qTo
        else:
            scaleFactor = float(self.stepSize) / self.distance(qFrom, qTo)
            npQFromCartesian = np.array(qFrom.cartesianParameters)
            npQToCartesian = np.array(qTo.cartesianParameters)
            npQFromAngle = np.array(qFrom.angleParameters)
            npQToAngle = np.array(qTo.angleParameters)
            qPrimeCartesian = npQFromCartesian + scaleFactor * (npQToCartesian - npQFromCartesian)
            qPrimeAngle = npQFromAngle + scaleFactor * (npQToAngle- npQFromAngle)
            qPrime = Configuration(list(qPrimeCartesian), list(qPrimeAngle))
            return qPrime