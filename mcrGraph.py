class MCRGraph():
    # vertices is a set of identifiers
    # edges is a hashmap (vertex -> [vertex1, vertex2,...])
    def __init__(self, vertices, edges):
        self.V = vertices
        self.E = edges
        self.vertexCovers = {}
        self.edgeCovers = {}
    
    def addVertex(self, vertex):
        assert(vertex not in self.V)
        assert(vertex not in self.E)
        self.V.add(vertex)
        self.E[vertex] = [] 

    #undirected graph so add edge from both points to other point
    def addEdge(self, point1, point2):
        assert(point1 in self.V)
        assert(point2 in self.V)
        assert(point1 in self.E)
        assert(point2 in self.E)
        self.E[point1].append(point2)
        self.E[point2].append(point1)

    def setVertexCover(self, q, cover):
        self.vertexCovers[q] = cover

    def getVertexCover(self, q):
        if q in self.vertexCovers:
            return self.vertexCovers[q]
        return None

    # sets the cover for the edge going both ways
    def setEdgeCover(self, q1, q2, cover):
        self.edgeCovers[(q1,q2)] = cover
        self.edgeCovers[(q2, q1)] = cover

    def getEdgeCover(self, q1, q2):
        if (q1,q2) in self.edgeCovers:
            return self.edgeCovers[(q1, q2)]
        return None

