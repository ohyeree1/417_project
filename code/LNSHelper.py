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
        if e() < p: 
            return x
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

def count_collisions(agent1, agent2): 
    """
    return all collisions between two agents given their location tables
    and the time that the last collision occurs
    
    Note: long edge collisions may end up counting multiple times, 
    but this does not affect the goal of having no collisions in the 
    final solution, only prioritises fixing these collisions more.
    """
        
    collisions = 0
    end_time = max(agent1[1],agent2[1])
    last = 0
    #collisions happening directly on a timestep 
    for i in range(end_time+1):
        a = get_location(agent1,i)
        b = get_location(agent2,i)
        if collided(a,b):
            collisions += 1
            last = max(i,last)
    
    #collisions happening between two timesteps
    #only possible case is when two agents swap nodes
    #via an edge of cost 1, higher cost is caught by direct timestep
    #TODO: adjust calculations for inbetween timestep collisions
    for j in range(end_time-1):
        a = get_location(agent1,j)
        aa = get_location(agent1,j+1)
        b = get_location(agent2,j)
        bb = get_location(agent2,j+1)
        if max(len(a),len(b),len(aa),len(bb)) == 1 and a == bb and b == aa: #edge 1 collision
            collisions += 1
            last = max(last,j+1)

    return collisions,last

def get_location(table,x): #return int location at time x given table
    if x >= table[1]: return table[0][table[1]] #agent has stopped moving
    else: return table[0][x]

def collided(x,y): 
    #x,y are the locations of two agents at a singular time
    #returns True if x,y result in collision, False otherwise
    if len(x) == 1 and len(y) == 1: #possible node collision, check if same
        return x == y
    elif len(x) == 2 and len(y) == 2: #possible edge collision, check if passthrough
        return (x[0] == y[1]) and (x[1] == y[0])
    return False # one agent is on a node and the other is on an edge, no collision

"""
def inbetween_collision(x,y):
"""

def determine_collisions(paths,agent,loc,move,time):
    # determine what collisions an action will result in for an agent
    collisions = list()
    tloc = [loc.ID,move[0].ID]
    for i in range(1,move[1]+1): #check each timestep
        if i == move[1]: tloc = [move[0].ID] #last step, at new node
        
        #collisions that occur directly on a timestep
        for j in range(len(paths)):
            if j != agent: #different paths
                aloc = get_location(paths[j],time+i)
                if collided(tloc,aloc): #collision occured
                    collisions.append([j,time+i])
        
    #collisions that occur inbetween two steps (only occurs with move[1] == 1 and not a wait move)
    if move[1] == 1 and loc != move[0]:
        for k in range(len(paths)):
            if k != agent: #different paths
                aloc = get_location(paths[j],time)
                bloc = get_location(paths[j],time+1)
                if max(len(aloc),len(bloc)) == 1 and aloc[0] == move[0].ID and bloc[0] == loc.ID: #inbetween collision
                    collisions.append([k,time+1])

    return collisions #list of [agent number,time of collision]

def create_location_table(path): #return a dict for where an agent is based on "time"
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

def find_path(my_map,h,paths,start,finish,agent,min_time):
    #returns a possible path for an agent based on a stocastic walk
    """
    my_map -> full graph object
    h -> heuristic function, enter any h[node] to get its cost
    paths -> list of location dictionaries for each agent
    start -> starting node
    finish -> goal node (h[finish] == 0)
    agent -> int value for the agent being fixed, ignore paths[agent] in collision detection
    min_time -> minimum amount of time the agent should take based on the collisions it had before
    """
    # initialize path with starting spot
    path = list() # list of nodes detailing the path
    path.append(start)
    loc = start

    # initialize collision tables
    np_collisions = [0]*len(paths)
    np_last_collision = [0]*len(paths)
    
    # While not at goal, look at all moves in position
    time = 0
    prev_wait = False #avoid potential case of getting stuck in an infinite wait loop
    while loc != finish or time < min_time:
        moveList = list()
        neighbours = list(loc.edges.keys())
        #print("keys:",neighbours,"starting loc:",loc.ID)
        for k in range(len(neighbours)):
            if neighbours[k] != loc.ID: moveList.append(loc.edges[neighbours[k]]) #[node,cost]

        if prev_wait: prev_wait = False
        else: moveList.append(loc.edges[loc.ID])
        
        # find which moves cause no collisions (including waiting), if any, else minimal collisions
        best_collision_count = 9999999999
        choices = list()
        for i in range(len(moveList)):
            move = moveList[i]
            move_collisions = determine_collisions(paths,agent,loc,move,time)
            cc = len(move_collisions)
            choices.append([cc,move[1]+h[move[0]],move[1],move[0].ID,move_collisions]) #TODO: attempt priority in moves
            """
            if cc != 0 and i+1 == len(moveList): break # do not consider wait if it causes a collision
            if cc < best_collision_count: #reset choice list, fewer collision option found
                best_collision_count = cc
                choices = list()
            if cc == best_collision_count: #candidate move, add to list
            
                
                A* = g(n)+h(n)
                g(n) = cost of move (current time is the same for each move so it can be removed)
                h(n) = heuristic (see h above)
                
                #[time+cost+heuristic, move_cost, move.ID, list of collisions caused]
                choices.append([move[1]+h[move[0]],move[1],move[0].ID,move_collisions])
            """
        # rank minimal collision moves by "A*" value
        choices.sort()

        # debug: what nodes are being chosen
        xxxxx = list()
        for iiiii in range(len(choices)):
            xxxxx.append(choices[iiiii][3])
        #print(*xxxxx,"neighbours ranked")


        # pseudorandomly pick one
        # If the first move trivially completes the path, choose it
        # Else, order the choices by best to worst and pseudorandomly choose one via weighted odds
        chosen_index = 0

        # for below, if choices[0][3] == finish.ID, then h[move[0]] = 0, 
        # thus first val is just the move cost 
        if choices[0][3] == finish.ID and time+choices[0][2] >= min_time: chosen_index = 0
        else: chosen_index = weighted_random(len(choices),0.8) #stocastically select, prioritize best option
        
        #add the new node to the path
        new_node = choices[chosen_index]
        time += new_node[2]
        #print("new loc:",new_node[3])
        if loc.ID == new_node[3]: prev_wait = True #wait action detected
        loc = my_map[new_node[3]] 
        path.append(loc)
        if len(path) > 100: #current error: only one move is considered viable waiting, causing an infinite loop
            #none of the graphs should have a path longer than 100 nodes, return empty if this is the case and try again
            return [],[],[]
        #update unavoidable collisions
        for v in range(len(new_node[4])):
            np_collisions[new_node[4][v][0]] += 1
            np_last_collision[new_node[4][v][0]] = max(np_last_collision[new_node[4][v][0]],new_node[4][v][1])
    # if at goal AND min time for the path is reached, then return the path

    return path,np_collisions,np_last_collision 