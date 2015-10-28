import random

def smoothPath(pathToSmooth, pathValidator):
    smoothed = list(pathToSmooth)
    #try and smooth len of list / 2 times
    for i in range(2 * len(pathToSmooth)):
        while True:
            if len(smoothed) < 3:
                return pathToSmooth
            j = random.randint(0, len(smoothed) - 1)
            k = random.randint(0, len(smoothed) - 1)
            if abs(j-k) >= 2:
                break
        smallerInd = min(j,k)
        largerInd = max(j,k)
        if pathValidator(smoothed[smallerInd], smoothed[largerInd]) == 0:
            # prune everything in the middle
            smoothed = smoothed[:smallerInd + 1] + smoothed[largerInd:]
    return smoothed

