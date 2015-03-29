import numpy as np
import math


def makeInitialTransforms(joints):
    referenceTransform = []
    inverseReferenceTransform = []
    for joint in joints:
        angle, jointLocation = joint
        radians = math.radians(angle)
        cosTheta = math.cos(radians)
        sinTheta = math.sin(radians)
        transform = np.array([[cosTheta, -sinTheta, jointLocation[0]], 
            [sinTheta, cosTheta, jointLocation[1]],
            [0, 0, 1]])
        referenceTransform.append(transform)
        inverseReferenceTransform.append(np.linalg.inv(transform))
    return referenceTransform, inverseReferenceTransform

def moveArm(configuration, referenceTransform, inverseReferenceTransform):
    transforms = []
    for jointNum, jointAngle in enumerate(configuration):
        radians = math.radians(jointAngle)
        cosTheta = math.cos(radians)
        sinTheta = math.sin(radians)
        rotationMatrix = np.array([[cosTheta, -sinTheta, 0],
                            [sinTheta, cosTheta, 0],
                            [0, 0, 1]])
        if jointNum == 0:
            transform = referenceTransform[jointNum].dot(rotationMatrix)
        else:
            previousJointTransform = transforms[jointNum - 1]
            transformToPreviousFrame = inverseReferenceTransform[jointNum - 1].dot(referenceTransform[jointNum])
            transform = previousJointTransform.dot(transformToPreviousFrame).dot(rotationMatrix)
        transforms.append(transform)
    for transform in transforms:
        print transform
    return transforms


joints = [(0, (1,1)), (0, (4,1)), (0, (7,1))]
referenceTransform, inverseReferenceTransform = makeInitialTransforms(joints)
configuration = [0, 90, -90]
transforms = moveArm(configuration, referenceTransform, inverseReferenceTransform)
for i, (_, point) in enumerate(joints):
    newPoint = transforms[i].dot(np.array([3, 0, 1]))
    print "{0} -> {1}".format(point, newPoint)
