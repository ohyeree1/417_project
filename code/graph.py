from random import randint,shuffle
from queue import PriorityQueue

"""
Helper file to define the graph structure
"""

#keeps track of a node's edges and their costs
class Node:
    def __init__(self,ID):
        self.ID = ID
        self.edges = {}
        self.h_value = 0
        # TO DO: add heuristics from compute heuristics
        self.printedges = {} #ONLY USED FOR SHOWING THE GRAPH, SHOULD NEVER BE USED IN ACTUAL COMPUTATION

    def __lt__(self, other):
        return self.h_value < other.h_value
        
    def add(self,x,c: int): # add edge to node x of cost c
        if self.edges.get(x.ID) != None: return False # max 1 edge between any 2 points
        self.edges[x.ID] = [x, c]   # Add neighboring node and the cost between them
        self.printedges[x.ID] = c
        return True # edge added

    def get_cost(self, neighbour_node):
        return self.edges[neighbour_node.ID]

    def __str__(self):
        return f'Node ID: {self.ID}, Edges: {self.printedges}'
        

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



#Graph consists of a list of Nodes with each Node's ID being its index in the list
class Graph:
    def __init__(self,nodeCount: int, agentCount: int): #nodes labelled 1 to n
        self.nodeCount = nodeCount #number of nodes
        self.agentCount = agentCount #number of agents
        self.nodeList = list() #list of nodes
        self.nodeList.append(None)
        for i in range(1,nodeCount+1):
            self.nodeList.append(Node(i))
        """
        generate random agents here
        agents are stored as list of (a,b),
        where a is start position and b is end position
        """
        self.agents = generateAgents(nodeCount,agentCount) #list of agents [start,finish], integer list
        
    def addEdge(self, a: int, b: int, c: int):
        if a == b: return False #no self edges allowed, waitcost is default to 1
        if a < 1 or a > self.nodeCount or b < 1 or b > self.nodeCount: return False #index out of range 
        if not self.nodeList[a].add(self.nodeList[b],c): return False #edge already added
        self.nodeList[b].add(self.nodeList[a],c)
        return True
