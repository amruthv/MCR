from mpl.algorithms.mcr import mcr
from mpl.algorithms.rrt_variants.rrt import new_rrt as rrt
from mpl.algorithms.rrt_variants.birrt import new_birrt as birrt
from mpl.algorithms.rrt_variants.birrt_ignore_start_goal_obstacles import birrt_ignore_obstacles 
from mpl.algorithms.rrt_variants.birrt_collision_based_removal import birrt_collision_based_removal as birrt_collision

# start is a list/tuple of parameters specifying configuration
# goal is a list/tuple of parameters specifying configuration
#
# 0 mcr
# 1 rrt
# 2 birrt
# 3 ignore start and goal birrt
# 4 collision based rrt
def runAlgorithm(start, goal, helper, algorithmNumber):
    if algorithmNumber == 0:
        algorithm = mcr.MCRPlanner(start, goal, helper)
    elif algorithmNumber == 1:
        print 'using just rrt'
        algorithm = rrt.RRTSearcher(start, goal, helper)
    elif algorithmNumber == 2:
        print 'using vanilla birrt'
        algorithm = birrt.BiRRTSearcher(start, goal, helper)
    elif algorithmNumber == 3:
        print 'using ignore start and goal'
        algorithm = birrt_ignore_obstacles.BiRRTIgnoreObstacleSearcher(start, goal, helper)
    elif algorithmNumber == 4:
        print 'using collision removal birrt'
        algorithm = birrt_collision.BiRRTCollisionRemovalSearcher(start, goal, helper, True)
    
    algorithm.run()
    path = algorithm.getPath()
    cover = algorithm.getCover()
    return (path, cover)