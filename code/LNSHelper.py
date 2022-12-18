#helper functions used in LNS search (stochastic variation)
from random import random as e

def weighted_random(choice_count: int, p: float) -> int:
    #given n choices labelled 0 to n-1, choose with a weighted distribution according to following:
    #chance of picking first value = p
    #chance of picking second value = p*(1-p)
    #chance of picking third value = p*(1-(p*1-p))
    #repeat until choice is found (or just give up and choose first one if no choice is found after arbritarily long)
    x = 0
    for i in range(choice_count*50):
        if e() < p: return x
        x = (x+1) % choice_count
    return 0


def cost_of_path(p): #return the cost of a single given path
    c = 0
    for k in range(len(p)-1):
        if p[k].ID == p[k+1].ID: #same node, wait
            c += 1
        else:
            c += p[k].edges[p[k+1].ID][1] #edge cost
    return c
    
def count_collisions(self, agent1, agent2): #return all collisions between two agents given their location tables
    collisions = 0
    end_time = max(agent1[1],agent2[1])
    #collisions happening directly on a timestep 
    for i in range(end_time):
        a = self.get_location(agent1,i)
        b = self.get_location(agent2,i)
        if a == b and len(a) == 1 and len(b) == 1: #vertex collision
            collisions += 1
        elif len(a) == 2 and len(b) == 2: #possible edge traversal collision
            if a[0] == b[1] and b[0] == a[1]: #edge traversal collision
                collisions += 1 
                """
                side effect is that for long edges the collision 
                may count multiple times, but does not affect the 
                goal of having no collisions in the final solution
                """
        #if lengths differ then no collision as one is one a vertex
        #and the other is traversing an edge

    #collisions happening between two timesteps
    #only possible case is when two agents swap nodes
    #via an edge of cost 1, higher cost is caught by direct timestep

    for j in range(end_time-1):
        a = self.get_location(agent1,j)
        aa = self.get_location(agent1,j+1)
        b = self.get_location(agent2,j)
        bb = self.get_location(agent2,j+1)
        if max(len(a),len(b),len(aa),len(bb)) == 1 and a == bb and b == aa: #edge 1 collision
            collisions += 1

    return collisions

def get_location(self,table,x): #return location at time x given table
    if x >= table[1]: return table[0][table[1]] #agent has stopped moving
    else: return table[0][x]

def create_location_table(self,path): #return a dict for where an agent is based on "time"
    #first value is a dictionary containing the locations, second value is an int for when the path ends 
    ans = {}
    time = 0
    for i in range(len(path)-1):
        ans[time] = [path[i].ID]
        if path[i].ID == path[i+1].ID: #wait
            time += 1
        else: #traversing edge
            edge_cost = path[i].edges[path[i+1].ID][1]
            for j in range(time+1,time+edge_cost):
                ans[j] = [path[i].ID,path[i+1].ID] #in process of traversing two nodes
            time += edge_cost
    ans[time] = [path[-1].ID] #last node
    return [ans,time] #after time agent will be at last node

def find_path(my_map,h,paths,start,finish,agent):
    #returns a possible path for an agent based on a stocastic walk
    """
    TODO: refactor collision methods to be out of the LNS object
    my_map -> full graph object
    h -> heuristic function, enter any h[node] to get its cost
    paths -> list of location dictionaries for each agent
    start -> starting node
    finish -> goal node (h[finish] == 0)
    agent -> int value for the agent being fixed, ignore paths[agent] in collision detection
    """
    path = list()
    return path