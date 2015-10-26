import math
from heapq import *

def distanceBetweenXYThetaPoints(point1, point2, penalizeRotation = False):
    distance = math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) **2 + (point2[2] - point1[2]) **2)
    return distance

def costOfPath(path):
    cost = 0
    for i in range(len(path)-1):
        cost += distanceBetweenXYThetaPoints(path[i], path[i+1])
    return cost

def reconstructPath(cameFrom, finalConfiguration):
    path = []
    location = finalConfiguration
    while location in cameFrom:
        path.insert(0, (location))
        location = cameFrom[location]
    path.insert(0, location)
    return path

def AStarSearch(startConfiguration, goal, adjacencyList):
    visited = set()
    cameFrom = {}
    gScore = {}
    fScore = {}
    gScore[startConfiguration] = 0
    fScore[startConfiguration] = gScore[startConfiguration] + distanceBetweenXYThetaPoints(startConfiguration, goal)

    priorityQueue = []
    priorityQueue.append([fScore[startConfiguration], startConfiguration])
   
    while len(priorityQueue) != 0:
        fScoreForPoint, currentConfiguration = heappop(priorityQueue)
        if (currentConfiguration[0], currentConfiguration[1]) == goal:
            return (reconstructPath(cameFrom, currentConfiguration), fScoreForPoint)
        
        visited.add(currentConfiguration)

        for neighbor in adjacencyList[currentConfiguration]:
            if neighbor in visited or currentConfiguration == neighbor:
                continue
            tentativeGScore = gScore[currentConfiguration] + distanceBetweenXYThetaPoints(currentConfiguration, neighbor, True)
            if neighbor not in priorityQueue or tentativeGScore < gScore[neighbor]:
                cameFrom[neighbor] = currentConfiguration
                gScore[neighbor] = tentativeGScore
                fScore[neighbor] = gScore[neighbor] + distanceBetweenXYThetaPoints(neighbor, goal)
                if neighbor not in priorityQueue:
                    heappush(priorityQueue, [fScore[neighbor], neighbor])
                heapify(priorityQueue)         
 
    return None
