import packagehelper

from mpl.common.world import World, SimpleObstacle

def getEmptyWorld():
    obstacles = []
    world = World(500, 500, obstacles)
    return world, obstacles

def getWorldMiddleObstacle():
    obstacle1 = SimpleObstacle("obs1", [(200,20), (300,20), (300,150), (200,150)])
    obstacles = [] 
    obstacles = [obstacle1]
    world = World(500,350, obstacles)
    return world, obstacles

def getManyObstaclesWorld():
    obstacle1 = SimpleObstacle("obs1", [(100,0), (275,0), (300,100), (190,180), (75, 100)], 4)
    obstacle2 = SimpleObstacle("obs2", [(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle("obs3", [(250, 200), (350, 200), (350, 375), (250, 375)], 20)
    obstacle4 = SimpleObstacle("obs4", [(375, 50), (480, 50), (480, 140), (375, 140)])
    obstacle5 = SimpleObstacle("obs5", [(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
    world = World(500,500, obstacles)
    return world, obstacles

def getTwoObstaclesDifferentWeightWorld():
    obstacle1 = SimpleObstacle("obs1", [(200,20), (300,20), (300,150), (200,150)], 4)
    obstacle2 = SimpleObstacle("obs2", [(200,150), (300,150), (300,500), (200,500)], 1)
    obstacles = [obstacle1, obstacle2]
    world = World(500,500, obstacles)
    return world, obstacles

def getManyObstaclesWeightedWorld():
    obstacle1 = SimpleObstacle("obs1", [(70,0), (245,0), (270,100), (160,180), (45, 100)])
    obstacle2 = SimpleObstacle("obs2", [(140, 385), (225,425), (140,480), (55,425)], 1)
    obstacle3 = SimpleObstacle("obs3", [(280, 200), (350, 200), (350, 375), (280, 375)], 4)
    obstacle4 = SimpleObstacle("obs4", [(375, 20), (480, 20), (480, 110), (375, 110)])
    obstacle5 = SimpleObstacle("obs5", [(410, 350), (430, 350), (430, 450), (410, 450), (410, 420), (360, 400), (410, 380)])
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5]
    world = World(500,500, obstacles)
    return world, obstacles

def getManySmallObstaclesWorld():
    obstacle1 = SimpleObstacle("obs1", [(70, 80), (130, 80), (130, 180), (70, 180)])
    obstacle2 = SimpleObstacle("obs2", [(70, 220), (130, 220), (130, 300), (70, 300)])
    obstacle3 = SimpleObstacle("obs3", [(70, 360), (130, 360), (130, 460), (70, 460)])
    obstacle4 = SimpleObstacle("obs4", [(190, 80), (250, 80), (250, 180), (190, 180)])
    obstacle5 = SimpleObstacle("obs5", [(190, 220), (250, 220), (250, 300), (190, 300)])
    obstacle6 = SimpleObstacle("obs6", [(190, 330), (250, 330), (250, 440), (190, 440)])
    obstacle7 = SimpleObstacle("obs7", [(320, 40), (480, 40), (480, 150), (320, 150)])
    obstacle8 = SimpleObstacle("obs8", [(320, 190), (480, 190), (480, 300), (320, 300)])
    obstacle9 = SimpleObstacle("obs9", [(320, 400), (480, 400), (480, 500), (320, 500)])
    obstacles = [obstacle1, obstacle2, obstacle3, \
                obstacle4, obstacle5, obstacle6, \
                obstacle7, obstacle8, obstacle9]
    world = World(500,500, obstacles)
    return world, obstacles


def get2DHandleAndCansWorld():
    sodaTop = SimpleObstacle("soda top", [(340, 300), (400, 300), (400, 350), (340, 350)], 1)
    sodaBot = SimpleObstacle("soda bot", [(300, 120), (400, 120), (400, 200), (300, 200)], 10)
    obstacles = [sodaTop, sodaBot]
    world = World(500, 500, obstacles)
    return world, obstacles

def get2DHandleAndClutteredWorld():
    sodaTop = SimpleObstacle("soda top", [(340, 300), (400, 300), (400, 350), (340, 350)], 1)
    sodaBot = SimpleObstacle("soda bot", [(300, 120), (400, 120), (400, 200), (300, 200)], 10)
    heavyBlock = SimpleObstacle("heavy block", [(130, 290), (250,290), (250, 400), (130, 400)], 20)
    longLightBlock = SimpleObstacle("long light", [(50, 250), (80,250), (80, 425), (50, 425)], 5)
    massiveBlock = SimpleObstacle("massive block", [(420, 0), (500,0), (500, 130), (420, 130)], 50)
    triangleBlock = SimpleObstacle("triangle", [(190, 30), (260, 30), (225, 150)], 3)
    smallBlock = SimpleObstacle("small block", [(120, 160), (230, 160), (230, 200), (120, 200)], 2)
    obstacles = [sodaTop, sodaBot, heavyBlock, longLightBlock, massiveBlock, triangleBlock, smallBlock]
    world = World(500, 500, obstacles)
    return world, obstacles

def get2DHandleAndTopLightClutteredWorld():
    sodaTop = SimpleObstacle("soda top", [(340, 300), (400, 300), (400, 350), (340, 350)], 4)
    sodaBot = SimpleObstacle("soda bot", [(300, 120), (400, 120), (400, 200), (300, 200)], 10)
    heavyBlock = SimpleObstacle("heavy block", [(130, 290), (250,290), (250, 400), (130, 400)], 5)
    longLightBlock = SimpleObstacle("long light", [(50, 250), (80,250), (80, 425), (50, 425)], 5)
    massiveBlock = SimpleObstacle("massive block", [(420, 0), (500,0), (500, 130), (420, 130)], 50)
    triangleBlock = SimpleObstacle("triangle", [(190, 30), (260, 30), (225, 150)], 10)
    smallBlock = SimpleObstacle("small block", [(210, 160), (280, 160), (280, 200), (210, 200)], 10)
    obstacles = [sodaTop, sodaBot, heavyBlock, longLightBlock, massiveBlock, triangleBlock, smallBlock]
    world = World(500, 500, obstacles)
    return world, obstacles

