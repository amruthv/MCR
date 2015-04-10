from mcrGraph import MCRGraph
import covercalculator
import numpy as np
import heapq

class MCRPlanner():
    # start and goal are both configurations
    def __init__(self, robot, world, start, goal, sim):
        self.robot = robot
        self.world = world
        self.start = start
        self.goal = goal
        self.cc = covercalculator.CoverCalculator(robot, world)
        self.sim = sim
        self.colorMap = {0: 'white', 1: 'black', 2: 'red', 3: 'green', 4: 'blue', 5: 'cyan', 6: 'purple', 7: 'yellow', 8 : 'orange'}
        self.nodeIds = {}
        self.G = self.initializeGraph()
        self.cameFrom = {}

    def discreteMCR(self, N_raise = 10):
        #setup stuff
        startCover = self.G.getLocalVertexCover(self.start)
        goalCover = self.G.getLocalVertexCover(self.goal)
        startGoalEdgeCover = self.cc.edgeCover(self.start, self.goal)
        s_min =  startCover.mergeWith(startGoalEdgeCover).mergeWith(goalCover).score
        print 'initial s_min', s_min
        k = startCover.score + goalCover.score
        print 'initial k =',k 
        G = self.G
        N = 1
        while True:
            self.expandRoadmap(G, k)
            self.cameFrom = self.computeMinExplanations(G)
            s_min = G.getTotalVertexCover(self.goal).score
            if N % N_raise == 0:
                print '====================='
                print "s_min = ", s_min
                print '====================='
                k += 1
            if k >= s_min:
                k = s_min - 1
            if N % N_raise == 0:
                print 'k = ', k
            N += 1
            if s_min == 0:
                print 'Got a collision free path'
                break
        print "took N = {0} iterations".format(N)
        print 'size of G: ', len(G.V)

    def initializeGraph(self):
        start = self.start
        goal = self.goal
        V = set()
        V.add(start)
        V.add(goal)
        E = {}
        E[start] = [goal]
        E[goal] = [start]
        graph = MCRGraph(V,E)
        startCover = self.cc.cover(start)
        graph.setLocalVertexCover(start, startCover)
        goalCover = self.cc.cover(goal)
        graph.setLocalVertexCover(goal, goalCover)
        startGoalEdgeCover = self.cc.edgeCover(start, goal)
        graph.setEdgeCover(start, goal, startGoalEdgeCover)
        graph.setTotalVertexCover(start, startCover)
        graph.setTotalVertexCover(goal, startCover.mergeWith(startGoalEdgeCover).mergeWith(goalCover))

        startId = self.sim.drawPoint(start)
        goalId = self.sim.drawPoint(goal)
        self.nodeIds[start] = startId
        self.nodeIds[goal] = goalId
        return graph

    def expandRoadmap(self, G, k, delta = 400.0):
        sampleConfig = self.robot.generateRandomConfiguration()
        nearestConfig = self.closest(G,k,sampleConfig)
        q = self.extendToward(G, nearestConfig, sampleConfig, delta, k)
        # couldn't find a point to extend towards satisfying k reachability
        if q is None:
            return
        # print self.robot.distance(q, nearestConfig)
        # s = self.sim.drawPoint((sampleConfig[0], sampleConfig[1]), fill = 'green')
        # self.robot.moveToConfiguration(sampleConfig)
        # ss = self.sim.drawRobot(self.robot, fill = 'green')
        # # print 'sampleConfig = ', sampleConfig
        # c = self.sim.drawPoint((nearestConfig[0], nearestConfig[1]), fill = 'black')
        # self.robot.moveToConfiguration(nearestConfig)
        # cc = self.sim.drawRobot(self.robot, fill = 'black')
        # # print 'nearestConfig =', nearestConfig
        # e = self.sim.drawPoint((q[0], q[1]), fill = 'purple')
        # self.robot.moveToConfiguration(q)
        # ee = self.sim.drawRobot(self.robot, fill = 'purple')
        # # raw_input()
        # self.sim.deleteObj(s)
        # self.sim.deleteObj(c)
        # self.sim.deleteObj(e)
        # self.sim.deleteManyObj(ss)
        # self.sim.deleteManyObj(cc)
        # self.sim.deleteManyObj(ee)
        neighborsOfQ = self.neighbors(G, q)
        if q not in G.V:
            qId = self.sim.drawPoint((q[0], q[1]))
            self.nodeIds[q] = qId
            G.addVertex(q)
            qCover = self.cc.cover(q)
            G.setLocalVertexCover(q, qCover)
            addedEdge = False
            for neighbor in neighborsOfQ:
                if round(self.robot.distance(neighbor, q)) <= round(delta): # and totalCover.score <= k:
                    addedEdge = True
                    G.addEdge(neighbor, q)
            assert(addedEdge == True) # should be true since the closestEdge should be within distance of delta

    def closest(self, G, k, sampleConfig):
        GKReachableNodes = []
        for node in G.V:
            if G.getTotalVertexCover(node).score <= k:
                GKReachableNodes.append(node)
        minIndex = np.argmin([self.robot.distance(q, sampleConfig) for q in GKReachableNodes])
        return GKReachableNodes[minIndex]

    def extendToward(self, G, closest, sample, delta, k, bisectionLimit = 4):
        # print 'closest = ', closest
        closestCover = G.getTotalVertexCover(closest)
        scaleFactor = min(delta / self.robot.distance(closest, sample), 1)
        scaledVector = scaleFactor * (np.array(sample) - np.array(closest))
        qPrime = np.array(closest) + scaledVector
        bisectionCount = 0
        while True:
            # print 'bisectionCount=', bisectionCount
            # print 'qPrime =', qPrime
            edgeCover = self.cc.edgeCover(closest, qPrime)
            qPrimeCover = self.cc.cover(qPrime)
            totalCover = closestCover.mergeWith(edgeCover).mergeWith(qPrimeCover)
            if totalCover.score <= k:
                return tuple(qPrime)
            elif bisectionCount < bisectionLimit:
                bisectionCount += 1
                qPrime = tuple(0.5 * (np.array(qPrime) + np.array(closest)))
            else:
                return None

    def neighbors(self, G, q, m = 10):
        # we choose nearest 10 rather than the ball of radius r approach
        distances = [(self.robot.distance(q, v), v) for v in G.V]
        sortedDistances = sorted(distances)
        neighbors = [x[1] for x in sortedDistances[:min(m, len(sortedDistances))]]
        return neighbors

    def computeMinExplanations(self, G, useGreedy = True):
        if useGreedy:
            return self.greedySearch(G)
        else:
            raise NotImplementedError()

    def greedySearch(self, G):
        # print "Number of edges to goal: ", len(G.E[self.goal])
        # raw_input()
        def coverScore(entry):
            return entry[0]
        def coverObj(entry):
            return entry[1]
        def entryId(entry):
            return entry[2]
        def makeEntry(coverObj, entryId):
            return [coverObj.score, coverObj, entryId]

        finalCovers = {}
        cameFrom = {}
        heap = []

        startCover = G.getTotalVertexCover(self.start)
        startEntry = makeEntry(startCover, self.start)
        coversSoFar = {self.start : startEntry}
        heapq.heappush(heap, startEntry)
        while len(finalCovers) < len(G.V):
            # find lowest unfinalized cover size node
            while True:
                entry = heapq.heappop(heap)
                vertexCover = coverObj(entry)
                vertexName = entryId(entry)
                if vertexName !=  "REMOVED":
                    del coversSoFar[vertexName]
                    break
            #finalize this distance
            finalCovers[vertexName] = vertexCover
            # add neighbors
            for neighbor in G.E[vertexName]:
                # only manipulate those not in finalCovers
                if neighbor not in finalCovers:
                    # solve neighbor cover with edge cover from vertexName
                    neighborCover = G.getLocalVertexCover(neighbor)
                    edgeCover = G.getEdgeCover(vertexName, neighbor)
                    if edgeCover == None:
                        edgeCover = self.cc.edgeCover(vertexName, neighbor)
                        G.setEdgeCover(vertexName, neighbor, edgeCover)
                    totalCover = vertexCover.mergeWith(edgeCover).mergeWith(neighborCover)
                    neighborEntry = makeEntry(totalCover, neighbor)
                    if neighbor not in coversSoFar:
                        coversSoFar[neighbor] = neighborEntry
                        heapq.heappush(heap, neighborEntry)
                        cameFrom[neighbor] = vertexName
                    elif totalCover.score < coverScore(coversSoFar[neighbor]):
                        # mark existing entry in heap as removed. Can just use the reference to the entry in the hashmap
                        coversSoFar[neighbor][2] = "REMOVED"
                        # override the value in dict of neighbor to new one
                        coversSoFar[neighbor] = neighborEntry
                        heapq.heappush(heap, neighborEntry)
                        cameFrom[neighbor] = vertexName
        
        # now go back through and fill in G.totalVertexCovers with the final covers
        G.clearTotalVertexCovers()
        for node in finalCovers:
            G.setTotalVertexCover(node, finalCovers[node])
        # self.updateColors()
        return cameFrom

    def updateColors(self):
        for node in self.G.V:
            coverScore = self.G.getTotalVertexCover(node).score
            self.sim.changePointFill(self.nodeIds[node], self.colorMap[coverScore])
