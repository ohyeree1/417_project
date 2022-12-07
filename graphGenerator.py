from random import randint,shuffle
from queue import PriorityQueue

"""
edge costs are calibrated to be random from 1 to 100
waiting cost for each node is random from 10 to 50
upper bound is to incentive waiting while lower bound is to
prevent each agent taking an "optimal" path 1 at a time

FILE FORMAT:

First line contains 3 values n,e,k
n is the number of nodes labelled 1 to n.
next e lines each contain a,b,c, where node a and node b are connected with cost c
next k lines contains vals s,g where agent starts at node s and ends at node g
"""


class Node:
    def __init__(self,ID):
        self.ID = ID
        self.edges = {}
        
    def add(self,x,c: int): #add edge to node x of cost c
        if self.edges.get(x.ID) != None: return False #max 1 edge between any 2 points
        self.edges[x.ID] = c
        return True #edge added


def generateAgents(n,a): #generate 'a' random agent paths for n nodes
    ar = [i for i in range(1,n+1)]
    br = [j for j in range(1,n+1)]
    shuffle(ar)
    shuffle(br)
    ar = ar[:a]
    agents = list()
    indexA = 0
    indexB = 0
    dup = list()
    while indexA != a:
        if ar[indexA] != br[indexB]:
            agents.append([ar[indexA],br[indexB]])
            indexA += 1
        else:
            dup.append(br[indexB])
        indexB += 1
        if indexB == len(br):
            indexB = 0
            br = dup
            dup = list()
    return agents
    
class Graph:
    def __init__(self,nodeCount: int, agentCount: int): #nodes labelled 1 to n
        self.nodeCount = nodeCount
        self.agentCount = agentCount
        self.nodeList = list()
        self.nodeList.append(None)
        for i in range(1,nodeCount+1):
            self.nodeList.append(Node(i))
        """
        generate random agents here
        agents are stored as list of (a,b),
        where a is start position and b is end position
        """
        self.agents = generateAgents(nodeCount,agentCount)
        
    def addEdge(self, a: int, b: int, c: int):
        if a == b: return False #no self edges allowed, waitcost is default to 1
        if a < 1 or a > self.nodeCount or b < 1 or b > self.nodeCount: return False #index out of range 
        if not self.nodeList[a].add(self.nodeList[b],c): return False #edge already added
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
    

def randomGraph(n: int, e: int, k: int):
    #return random connected graph with n nodes, e edges and k agents
    if e < n-1 or e > (n*n-n)//2: return None #impossible
    if k > n: return None #too many agents
    p = prufer(n)
    startingEdges = decodePrufer(p)
    g = Graph(n,k)
    for i in range(len(startingEdges)):
        c = randint(1,5) #next edge time cost
        assert g.addEdge(startingEdges[i][0],startingEdges[i][1],c)

    #g is now a tree
    extraEdges = e-n+1
    while extraEdges != 0:
        a = randint(1,n)
        b = randint(1,n)
        c = randint(1,5) #cost of next edge
        if g.addEdge(a,b,c): #new edge added if True
            extraEdges -= 1
    return g

def alternateGraph(n: int, e: int, k: int):
    """
    alternate generator that guarantees each agent's path
    is at least 8 nodes long (will likely be more)
    starting points are nodes 1-k
    end points are nodes (n-k+1)-n
    an edge can only connect two nodes with difference at most n//10
    """
    g = Graph(n,k)

    #configure agents
    ar = [a for a in range(1,k+1)]
    br = [b for b in range(n+1-k,n+1)]
    shuffle(ar)
    shuffle(br)
    cr = list()
    for ii in range(len(ar)):
        cr.append([ar[ii],br[ii]])
    g.agents = cr
    
    #create basic tree (1-2-3-4-5...-n)
    for i in range(n-1):
        g.addEdge(i+1,i+2,randint(1,5))
    extraEdges = e-n+1
    edgeLimit = n//10
    while extraEdges != 0:
        a = randint(1,n)
        b = a + randint(2,edgeLimit) #connect at most 1/10th of the graph apart
        c = randint(1,5) #cost of next edge
        if g.addEdge(a,b,c): #new edge added if True
            extraEdges -= 1
    return g

def writeGraph(fileName: str, graph: Graph):
    nc = graph.nodeCount
    ac = graph.agentCount
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
    #first line
    line = str(nc)+" "+str(ec)+" "+str(ac)+"\n"
    f.write(line)
    
    #edges
    for k in range(ec):
        line = str(edges[k][0])+" "+str(edges[k][1])+" "+str(edges[k][2])+"\n" 
        f.write(line)

    #agents
    for l in range(ac):
        line = str(graph.agents[l][0])+" "+str(graph.agents[l][1])+"\n"
        f.write(line)
    f.close()

def readGraph(fileName: str):
    """
    create a graph from a given file
    update this later to directly create the agents
    instead of file -> array -> agents
    """
    f = open(fileName,"r")
    line = f.readline()
    nodes,edges,agents = map(int,line.split(" "))
    g = Graph(nodes,agents)

    
    #add edges
    for i in range(edges):
        line = f.readline()
        a,b,c = map(int,line.split(" "))
        g.addEdge(a,b,c)

    #adjust agents
    for w in range(agents):
        line = f.readline()
        a,b = map(int,line.split(" "))
        g.agents[w][0] = a
        g.agents[w][1] = b
    f.close()
    return g

if __name__ == "__main__":
    #create graphs here
    test = randomGraph(10,20,3)
    writeGraph("generatedGraphs/test.txt",test)
    test2 = readGraph("generatedGraphs/test.txt")
    writeGraph("generatedGraphs/test2.txt",test2)
    #test.txt and test2.txt should be the same

    # creates 60 tests
    agents = [2,10,50] #number of agents
    graphSize = [10,100,1000] #number of nodes
    edgeMultiplier = [1.1,1.25,1.5,2,3] #number of edges relative to nodes
    for i in range(len(graphSize)):
        for j in range(len(edgeMultiplier)):
            for k in range(4):
                name = "generatedGraphs/graph"+str(i*20+j*4+k+1)+".txt"
                writeGraph(name,randomGraph(graphSize[i],round(graphSize[i]*edgeMultiplier[j]),agents[i]))

    #20 extra tests of intentionally designed tests to prevent very short routes
    """
    100 nodes -> 5, 10 agents
    1000 nodes -> 50, 100 agents
    all use 1.5 edge multiplier, avg(deg(node)) = 3
    """
    graphSize = [100,1000]
    agentRatio = [20,10]
    edgeMultiplier = 1.5
    for t in range(2):
        for u in range(2):
            for v in range(5):
                name = "generatedGraphs/graph"+str(t*10+u*5+v+61)+".txt"
                writeGraph(name,alternateGraph(graphSize[t],round(graphSize[t]*edgeMultiplier),graphSize[t]//agentRatio[u]))

    
    
