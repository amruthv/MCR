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
        self.G = self.initializeGraph()
        self.sim = sim
        self.cameFrom = {}

    def discreteMCR(self, N_raise = 10):
        #setup stuff
        startCover = self.cc.cover(self.start)
        goalCover = self.cc.cover(self.goal)
        startGoalEdgeCover = self.cc.edgeCover(self.start, self.goal)
        s_min =  startCover.mergeWith(startGoalEdgeCover).score
        print 'initial s_min', s_min
        k = startCover.score + goalCover.score
        print 'initial k =',k 
        G = self.G
        N = 1
        while True:
            self.expandRoadmap(G, k)
            # print 'G.V', G.V
            # print 'G.E', G.E
            self.cameFrom = self.computeMinExplanations(G)
            s_min = G.covers[self.goal].score
            if N % 10 == 0:
                print '====================='
                print "s_min = ", s_min
                print '====================='
            if N % N_raise == 0:
                k += 1
                print 'k = ', k
            if k >= s_min:
                k = s_min - 1
            N += 1
            if s_min == 0:
                # print 'Got a collision free path'
                break
            # print G.covers
        print "took N = {0} iterations".format(N)

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
        graph.covers[start] = self.cc.cover(start)
        graph.covers[goal] = graph.covers[start].mergeWith(self.cc.edgeCover(start, goal))  # do i need to set this
        return graph

    def expandRoadmap(self, G, k, delta = 300):
        sampleConfig = self.robot.generateRandomConfiguration()
        # print 'sampleConfig = ', sampleConfig
        nearestConfig = self.closest(G,k,sampleConfig)
        q = self.extendToward(G, nearestConfig, sampleConfig, delta, k)
        # print 'extendedTowardSample = ', q
        neighborsOfQ = self.neighbors(G, q)
        for neighbor in neighborsOfQ:
            if self.robot.distance(neighbor, q) < delta:
                if q not in G.V:
                    G.addVertex(q)
                    self.sim.drawPoint((q[0], q[1]))
                G.addEdge(neighbor, q)

    def closest(self, G, k, sampleConfig):
        GKReachableNodes = []
        for node in G.V:
            if G.covers[node].score <= k:
                GKReachableNodes.append(node)
        minIndex = np.argmin([self.robot.distance(q, sampleConfig) for q in GKReachableNodes])
        return GKReachableNodes[minIndex]

    def extendToward(self, G, closest, sample, delta, k, bisectionLimit = 4):
        # print 'extendToward k= ', k
        closestCover = G.covers[closest]
        qPrime = tuple(np.array(closest) + 
                min(float(delta) / self.robot.distance(closest, sample), 1) * 
                    np.array(sample) - np.array(closest))
        bisectionCount = 0
        while True:
            edgeCover = self.cc.edgeCover(closest, qPrime)
            qPrimeCover = closestCover.mergeWith(edgeCover)
            if qPrimeCover.score <= k:
                break
            elif bisectionCount < bisectionLimit:
                # print 'BROKE THE LIMIT FOR K IN EXTENDING with qPrimeCover score = ', qPrimeCover.score
                bisectionCount += 1 
                qPrime = tuple(0.5 * (np.array(qPrime) + np.array(closest)))
            else:
                break
        return qPrime

    def neighbors(self, G, q, m = 10):
        # we choose nearest 10 rather than the ball of radius r approach
        distances = [(self.robot.distance(q, v), v) for v in G.V]
        sortedDistances = sorted(distances)
        neighbors = [x[1] for x in sortedDistances[:min(m, len(sortedDistances))]]
        return neighbors

    def computeMinExplanations(self, G, useGreedy = True):
        if useGreedy:
            return self.greedySearch(G, self.start)
        else:
            raise NotImplementedError()

    def greedySearch(self, G, start):
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

        startCover = self.cc.cover(start)
        startEntry = makeEntry(startCover, start)
        coversSoFar = {start : startEntry}
        heapq.heappush(heap, startEntry)
        while len(finalCovers) < len(G.V):
            # print 'finalCovers = ', finalCovers
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
                    # if neighbor == self.goal:
                        # print 'vertexName = ', vertexName
                        # print 'vertexName cover score = ', vertexCover.score
                    # solve neighbor cover with edge cover from vertexName
                    neighborCover = vertexCover.mergeWith(self.cc.edgeCover(vertexName, neighbor))
                    neighborEntry = makeEntry(neighborCover, neighbor)
                    if neighbor not in coversSoFar:
                        coversSoFar[neighbor] = neighborEntry
                        heapq.heappush(heap, neighborEntry)
                        cameFrom[neighbor] = vertexName
                    elif neighborCover.score < coverScore(coversSoFar[neighbor]):
                        # mark existing entry in heap as removed. Can just use the reference to the entry in the hashmap
                        coversSoFar[neighbor][2] = "REMOVED"
                        # override the value in dict of neighbor to new one
                        coversSoFar[neighbor] = neighborEntry
                        heapq.heappush(heap, neighborEntry)
                        cameFrom[neighbor] = vertexName
        
        # now go back through and fill in G.Covers with the final covers
        G.covers = {}
        for node in finalCovers:
            G.covers[node] = finalCovers[node]

        return cameFrom
