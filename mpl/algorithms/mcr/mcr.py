import numpy as np
import heapq
import pdb
from mcrGraph import MCRGraph
from mpl.common.covercalculator import CoverCalculator
from mpl.common import searcher

class MCRPlanner():
    # start and goal are both configurations
    def __init__(self, start, goal, helper, useTLPObstacles, numIterations = 100, verbose = False, sim = None, shouldDraw = False):
        self.start = start
        self.goal = goal
        self.helper = helper
        self.useTLPObstacles = useTLPObstacles
        self.cc = CoverCalculator(helper, useTLPObstacles)
        self.numIterations = numIterations
        self.verbose = verbose
        self.sim = sim
        self.shouldDraw = shouldDraw
        self.G = self.initializeGraph()
        self.cameFrom = {}
        self.colorMap = {0: 'white', 1: 'black', 2: 'red', 3: 'green', 4: 'blue', 5: 'cyan', 6: 'purple', 7: 'yellow', 8 : 'orange'}
        self.nodeIds = {}

    def run(self):
        return self.discreteMCR()

    def discreteMCR(self, N_raise = 10):
        #setup stuff
        startCover = self.cc.cover(self.start)
        goalCover = self.cc.cover(self.goal)
        best_possible_cover = startCover.mergeWith(goalCover)
        startGoalEdgeCover = self.cc.edgeCover(self.start, self.goal)
        s_min =  best_possible_cover.mergeWith(startGoalEdgeCover).score
        best_possible_score = best_possible_cover.score
        k = startCover.score + goalCover.score
        if self.verbose:
            print 'initial s_min', s_min
            print 'initial k =', k 
        G = self.G
        print 'original s_min', s_min
        iterCount = self.numIterations
        for i in range(self.numIterations):
            self.expandRoadmap(G, k)
            self.cameFrom = self.computeMinExplanations(G)
            s_min = G.getTotalVertexCover(self.goal).score
            if i % N_raise == 0:
                if self.verbose:
                    print '====================='
                    print "s_min = ", s_min
                    print '====================='
                k += 1
            if k >= s_min:
                k = s_min - 1
            if self.verbose and i % N_raise == 0:
                print 'k = ', k
                print 's_min', s_min
            if s_min == best_possible_score:
                iterCount = i
                break
        if self.verbose:
            if s_min == best_possible_score:
                print 'Got best path possible with score', s_min
            else:
                print 'Got subpar path of score {0} when best score possible is {1}'.format(s_min, best_possible_score)
            print "took {0} iterations".format(iterCount)
            print 'size of G: ', len(G.V)
        return s_min

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
        startGoalEdgeCover = self.cc.edgeCover(start, goal)
        graph.setEdgeCover(start, goal, startGoalEdgeCover)
        graph.setTotalVertexCover(start, startCover)
        graph.setTotalVertexCover(goal, startGoalEdgeCover)
        if self.shouldDraw:
            startId = self.sim.drawPoint(start)
            goalId = self.sim.drawPoint(goal)
            self.nodeIds[start] = startId
            self.nodeIds[goal] = goalId
        return graph

    def expandRoadmap(self, G, k):
        sampleConfig = self.helper.sampleConfig(self.goal)
        nearestConfig = self.closest(G,k,sampleConfig)
        q = self.extendToward(G, nearestConfig, sampleConfig, k)
        # couldn't find a point to extend towards satisfying k reachability
        if q is None:
            return
        if q == self.goal:
            return
        if q not in G.V:
            if self.shouldDraw:
                qId = self.sim.drawPoint((q[0], q[1]))
                self.nodeIds[q] = qId
            G.addVertex(q)
            addedEdge = False
            neighborsOfQ = self.neighbors(G, q)
            neighborsOfQ = [qq for qq in neighborsOfQ if qq != q]
            for neighbor in neighborsOfQ:
                # if its within the extendable distance
                qq = self.helper.stepTowards(neighbor, q)
                if all(x-y < 1.0e-6 for x,y in zip(q, qq)):
                # if self.helper.stepTowards(neighbor, q) == q: # and totalCover.score <= k:
                    addedEdge = True
                    G.addEdge(neighbor, q)
            try:
                assert(addedEdge == True) # should be true since the closestEdge should be within distance of stepSize
            except:
                pdb.set_trace()
                print 'something went wrong'

    def closest(self, G, k, sampleConfig):
        GKReachableNodes = []
        for node in G.V:
            if G.getTotalVertexCover(node).score <= k:
                GKReachableNodes.append(node)
        distances = [self.helper.distance(q, sampleConfig) if not (sampleConfig == self.goal and q == self.goal) else float('inf') for q in GKReachableNodes]
        minIndex = np.argmin(distances)
        return GKReachableNodes[minIndex]

    def extendToward(self, G, closest, sample, k):
        qPrime = self.helper.stepTowards(closest, sample)
        closestCover = G.getTotalVertexCover(closest)
        edgeCover = self.cc.edgeCover(closest, qPrime)
        totalCover = closestCover.mergeWith(edgeCover)
        if totalCover.score <= k:
            return qPrime
        else:
            return None

    def neighbors(self, G, q, m = 10):
        # we choose nearest 10 rather than the ball of radius r approach
        distances = [(self.helper.distance(q, v), v) for v in G.V]
        sortedDistances = sorted(distances)
        neighbors = [x[1] for x in sortedDistances[:min(m, len(sortedDistances))]]
        return neighbors

    def computeMinExplanations(self, G, useGreedy = True):
        if useGreedy:
            return self.greedySearch(G)
        else:
            raise NotImplementedError()

    def greedySearch(self, G):
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
                    edgeCover = G.getEdgeCover(vertexName, neighbor)
                    if edgeCover == None:
                        edgeCover = self.cc.edgeCover(vertexName, neighbor)
                        G.setEdgeCover(vertexName, neighbor, edgeCover)
                    totalCover = vertexCover.mergeWith(edgeCover)
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

    def getPath(self):
        if self.useTLPObstacles and 'permanent' in self.getCover():
            return []
        path = searcher.reconstructPath(self.cameFrom, self.goal)
        return path

    def getCover(self):
        return list(self.G.getTotalVertexCover(self.goal).cover)

    def updateColors(self):
        for node in self.G.V:
            coverScore = self.G.getTotalVertexCover(node).score
            self.sim.changePointFill(self.nodeIds[node], self.colorMap[coverScore])
