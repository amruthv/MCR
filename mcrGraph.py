class MCRGraph():
    # vertices is a set of identifiers
    # edges is a hashmap (vertex -> [vertex1, vertex2,...])
    def __init__(self, vertices, edges):
        self.V = vertices
        self.E = edges
        self.covers = {}
    
    def addVertex(self, vertex):
        self.V.add(vertex)

    #undirected graph so add edge from both points to other point
    def addEdge(self, point1, point2):
        assert(point1 in self.V)
        assert(point2 in self.V)
        assert(point1 in self.E)
        assert(point2 in self.E)
        self.E[point1].append(point2)
        self.E[point2].append(point1)

