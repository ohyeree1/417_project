from random import randint
from queue import PriorityQueue

class Node:
    def __init__(self,ID):
        self.ID = ID
        self.edges = {}
        
    def add(self,x,c: int): #add edge to node x of cost c
        if self.edges.get(x.ID) != None: return False #max 1 edge between any 2 points
        self.edges[x.ID] = c
        return True #edge added


class Graph:
    def __init__(self,nodeCount: int): #nodes labelled 1 to n
        self.nodeCount = nodeCount
        self.nodeList = list()
        self.nodeList.append(None)
        for i in range(1,nodeCount+1):
            self.nodeList.append(Node(i))

    def addEdge(self, a: int, b: int, c: int):
        if a == b: return False #no self edges allowed
        if not self.nodeList[a].add(self.nodeList[b],c): return False #invalid edge
        self.nodeList[b].add(self.nodeList[a],c)
        return True

def prufer(n: int):
    #return random prufer code for n nodes
    ar = list()
    for i in range(n-2):
        ar.append(randint(1,n))
    return ar

def decodePrufer(p: list): #return edges based on purfer code
    degree = [1]*(len(p)+3)
    degree[0] = 0
    for i in range(len(p)):
        degree[p[i]] += 1
    maxHeap = PriorityQueue()
    for j in range(len(degree)):
        if degree[j] == 1: maxHeap.put((-j,j)) #(key,value), pqueue prioritises lowest val
    edges = list()
    for k in range(len(p)):
        a = p[k]
        b = maxHeap.get()[1]
        edges.append([a,b])
        degree[a] -= 1
        degree[b] -= 1 # = 0
        if degree[a] == 1: #new leaf
            maxHeap.put((-a,a))
    #should have two leaves left in maxHeap now
    a = maxHeap.get()[1]
    b = maxHeap.get()[1]
    edges.append([a,b])
    return edges
    

def randomGraph(n: int, e: int):
    #return random connected graph with n nodes and e edges
    if e < n-1 or e > (n*n-n)//2: return None #impossible
    p = prufer(n)
    startingEdges = decodePrufer(p)
    g = Graph(n)
    for i in range(len(startingEdges)):
        c = randint(1,100) #next edge cost
        assert g.addEdge(startingEdges[i][0],startingEdges[i][1],c)

    #g is now a tree
    extraEdges = e-n+1
    while extraEdges != 0:
        a = randint(1,n)
        b = randint(1,n)
        c = randint(1,100) #cost of next edge
        if g.addEdge(a,b,c): #new edge added?
            extraEdges -= 1
    return g

def writeGraph(fileName: str, graph: Graph):
    nc = graph.nodeCount
    #find and list edges
    edges = list() #[node a, node b, cost]
    for i in range(1,nc+1):
        node = graph.nodeList[i]
        neighbours = list(node.edges.keys())
        for j in range(len(neighbours)):
            n = neighbours[j]
            if i < n: edges.append([i,n,node.edges[n]])
    ec = len(edges)
    f = open(fileName,"w")
    line = str(nc)+" "+str(ec)+"\n"
    f.write(line)
    for k in range(ec):
        line = str(edges[k][0])+" "+str(edges[k][1])+" "+str(edges[k][2])+"\n" 
        f.write(line)
    f.close()

def readGraph(fileName: str): #create a graph from a given file
    f = open(fileName,"r")
    line = f.readline()
    nodes,edges = map(int,line.split(" "))
    g = Graph(nodes)
    for i in range(edges):
        line = f.readline()
        a,b,c = map(int,line.split(" "))
        g.addEdge(a,b,c)
    f.close()
    return g

if __name__ == "__main__":
    #create graphs here
    test = randomGraph(10,20)
    writeGraph("generatedGraphs/test.txt",test)
    test2 = readGraph("generatedGraphs/test.txt")
    writeGraph("generatedGraphs/test2.txt",test2)
    #test.txt and test2.txt should be the same

    #create 80 tests according to specifications
    # 5 graphs for each setting
    graphSize = [10,100,1000,10000] #number of nodes
    edgeMultiplier = [1.1,1.25,1.5,2,3] #number of edges relative to nodes
    for i in range(len(graphSize)):
        for j in range(len(edgeMultiplier)):
            for k in range(4):
                name = "generatedGraphs/graph"+str(i*20+j*4+k+1)+".txt"
                writeGraph(name,randomGraph(graphSize[i],round(graphSize[i]*edgeMultiplier[j])))
    #20 extra tests of intentionally designed tests to prevent very short routes
    #to be added: agents and goals, alternate generator to guarantee no short paths


    
    
