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

def count_collisions(agent1, agent2, node_list): 
    """
    return the number of collisions between two agents given their location tables
    and the time that the last collision occurs
    """
        
    collisions = 0
    end_time = max(agent1[1],agent2[1])
    last = 0
    #collisions happening directly on a timestep 
    for i in range(end_time+1):
        a = get_location(agent1,i)
        b = get_location(agent2,i)
        if single_collide(a,b,node_list):
            collisions += 1
            last = max(i,last)
    
    #collisions happening between two timesteps
    for j in range(end_time):
        a = get_location(agent1,j)
        aa = get_location(agent1,j+1)
        b = get_location(agent2,j)
        bb = get_location(agent2,j+1)
        if multi_collide(a,aa,b,bb,node_list): #collision occured
            collisions += 1
            last = max(last,j+1)

    return collisions,last

def get_location(table,x): #return int location at time x given table
    if x >= table[1]: return table[0][table[1]] #agent has stopped moving
    else: return table[0][x]

def single_collide(x,y,node_list): 
    #x,y are the locations of two agents at a singular time
    #returns True if x,y result in collision, False otherwise

    if x == y: return True #either node collision or traversing same edge in same direction at same time
    elif len(x) == 3 and len(y) == 3: #possible edge collision, check if on the same spot in the edge
        return (x[0] == y[1]) and (x[1] == y[0]) and ((x[2] + y[2]) == node_list[x[0]].edges[x[1]][1])
    return False # one agent is on a node and the other is on an edge, no collision

def multi_collide(x0,x1,y0,y1,node_list):
    #given agents x and y at two consecutive time steps, determine if they collide
    #node to node passthrough
    if max(len(x0),len(x1),len(y0),len(y1)) == 1:
        return (x0 == y1) and (y0 == x1)
    #edge to edge passthrough
    if min(len(x0),len(x1),len(y0),len(y1)) == 3:
        return (x0[0] == y0[1]) and (x0[1] == y0[0]) and ((x0[2]+x1[2]+y0[2]+y1[2]) == (node_list[x0[0]].edges[x0[1]][1]*2))
    #node to edge passthrough
    if len(x0) == 1 and len(x1) == 3 and len(y0) == 3 and len(y1) == 1:
        return (x0 == y1) and (x1[0] == y0[1]) and (x1[1] == y0[0])
    #edge to node passthrough
    if len(x0) == 3 and len(x1) == 1 and len(y0) == 1 and len(y1) == 3:
        return (x1 == y0) and (x0[0] == y1[1]) and (x0[1] == y1[0])
    return False

def determine_collisions(paths,agent,loc,move,time,node_list):
    # determine what collisions an action will result in for an agent
    """
    paths -> list of location dictionaries for each agent
    agent -> agent the move is for, ignore this one
    loc -> current location
    move -> move to be made, [node moving to,cost]
    time -> current time
    node_list -> list of nodes that make up the graph
    """
    collisions = list() #list of all collisions [agent.ID,time]
    spots = list() # location for agent if move is made from time to time+cost inclusive
    spots.append([loc.ID])
    for i in range(1,move[1]):
        spots.append([loc.ID,move[0].ID,i])
    spots.append([move[0].ID])

    for j in range(len(paths)):
        if j != agent: #differing agents
            for k in range(1,move[1]+1): #direct collision
                a = get_location(paths[j],time+k)
                if single_collide(spots[k],a,node_list):
                    collisions.append([j,time+k])
            for l in range(move[1]): #indirect collision
                a = get_location(paths[j],time+l)
                b = get_location(paths[j],time+l+1)
                if multi_collide(a,b,spots[l],spots[l+1],node_list):
                    collisions.append([j,time+l+1])
                
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
                ans[j] = [path[i].ID,path[i+1].ID,j-time] #in process of traversing two nodes
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
        for k in range(len(neighbours)):
            if neighbours[k] != loc.ID: moveList.append(loc.edges[neighbours[k]]) #[node,cost]

        if prev_wait: prev_wait = False
        else: moveList.append(loc.edges[loc.ID])
        
        # find important info about each move [# collisions caused,A* value,node movinrg to, collisions caused]
        choices = list()
        for i in range(len(moveList)):
            move = moveList[i]
            move_collisions = determine_collisions(paths,agent,loc,move,time,my_map)
            cc = len(move_collisions)
            choices.append([cc,move[1]+h[move[0]],move[1],move[0].ID,move_collisions]) #TODO: attempt priority in moves
            """
            above priorities are in order of the following:
            1. number of collisions
            2. "A*" value (move cost+heuristic, time is always same, no need to add)
            3. move cost
            4. ID of node moving to
            move_collisions is kept for later use below
            """
        # rank minimal collision moves by "A*" value
        choices.sort()


        # pseudorandomly pick one
        # If the first move trivially completes the path, choose it
        # Else, order the choices by best to worst and pseudorandomly choose one via weighted odds
        chosen_index = 0

        # for below, if choices[0][3] == finish.ID, then h[move[0]] = 0, 
        # thus first val is just the move cost 
        if choices[0][3] == finish.ID and time+choices[0][2] >= min_time: chosen_index = 0 #trivial completion
        else: chosen_index = weighted_random(len(choices),0.9) #stocastically select, prioritize best option
        
        #add the new node to the path
        new_node = choices[chosen_index]
        time += new_node[2]
        #print("new loc:",new_node[3])
        if loc.ID == new_node[3]: prev_wait = True #wait action detected
        loc = my_map[new_node[3]] 
        path.append(loc)
        if len(path) > 100: 
            #none of the graphs should have a path longer than 100 nodes, return empty if this is the case and try again
            return [],[],[]
        #update collisions
        for v in range(len(new_node[4])):
            np_collisions[new_node[4][v][0]] += 1
            np_last_collision[new_node[4][v][0]] = max(np_last_collision[new_node[4][v][0]],new_node[4][v][1])
    
    # if at goal AND min time for the path is reached, then return the path
    return path,np_collisions,np_last_collision 
