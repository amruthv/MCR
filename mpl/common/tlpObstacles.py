OBSTACLE_WEIGHTS = {'permanent': float('inf'), 'shadow': 2, 'obstacle': 10}

def getShadowFromObstacle(obstacleName):
    return obstacleName + "_shadow"