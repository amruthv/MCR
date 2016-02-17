import math
import numpy as np
import random

class Configuration():
    def __init__(self, cartesianParameters, angleParameters):
        self.cartesianParameters = cartesianParameters
        self.angleParameters = angleParameters

    def __str__(self):
        return 'Cartesian: {0}, Angles: {1}'.format(self.cartesianParameters, self.angleParameters)

    def __hash__(self):
        return tuple(self.cartesianParameters + self.angleParameters).__hash__()

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.cartesianParameters == other.cartesianParameters and self.angleParameters == other.angleParameters
        return False

    def getLastAngle(self):
        return self.angleParameters[:-1]

    def toPrimitive(self):
        return tuple(self.cartesianParameters + self.angleParameters)

    @staticmethod
    def getRandomConfiguration(cartesianMinX, cartesianMaxX, cartesianMinY, cartesianMaxY, numJoints):
        cartesianParameters = [random.uniform(cartesianMinX, cartesianMaxX), random.uniform(cartesianMinY, cartesianMaxY)]
        angleParameters = [random.uniform(-math.pi, math.pi) for i in range(numJoints)]
        return Configuration(cartesianParameters, angleParameters)

    @staticmethod
    def generateInBetweenConfigurations(q_from, q_to, stepsToCheck = 40):
        q_from_cartesian = q_from.cartesianParameters
        q_to_cartesian = q_to.cartesianParameters
        cartesianGenerators = []
        angleGenerators = []
        for i in range(len(q_from_cartesian)):
            cartesianGenerators.append(Configuration.normalInterpolateGenerator(q_from_cartesian[i], q_to_cartesian[i], stepsToCheck))
        q_from_angles = q_from.angleParameters
        q_to_angles = q_to.angleParameters
        for i in range(len(q_from_angles)):
            angleGenerators.append(Configuration.angleGenerator(q_from_angles[i], q_to_angles[i], stepsToCheck))
        for i in range(stepsToCheck):
            nextCartesianParams = [g.next() for g in cartesianGenerators]
            nextAngleParams = [g.next() for g in angleGenerators]
            yield Configuration(nextCartesianParams, nextAngleParams)

    @staticmethod
    def angleGenerator(q_from, q_to, stepsToCheck):
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
            if (positiveAngle - negativeAngle) > (twoPiShiftedNegativeAngle - positiveAngle):
                # we should use the addition of 2pi
                if positiveAngle == q_from:
                    qFrom = positiveAngle
                    qTo = twoPiShiftedNegativeAngle
                else:
                    qFrom = twoPiShiftedNegativeAngle
                    qTo = positiveAngle
                return Configuration.normalInterpolateGenerator(qFrom, qTo, stepsToCheck, True)
        return Configuration.normalInterpolateGenerator(q_from, q_to, stepsToCheck, False)

    # assumes that q_from and q_to are singular scalars
    @staticmethod
    def normalInterpolateGenerator(q_from, q_to, stepsToCheck, needsAngleAdjustment = False):
        q_delta = q_to - q_from
        for i in range(0, stepsToCheck + 1):
            q = q_from + q_delta * float(i) / stepsToCheck
            if needsAngleAdjustment and q > math.pi:
                yield q - 2 * math.pi
            yield q
