import heapq
from graph import *

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(node_list, goal_node):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal_node, 'cost': 0}
    visited = []
    heapq.heappush(open_list, (root['cost'], goal_node, root, visited))

    # We are traversing back from the goal node
    closed_list[goal_node] = root
    while len(open_list) > 0:
        (cost, curr_node, curr, visited) = heapq.heappop(open_list)

        neighbors = curr_node.edges
        for neighbor in neighbors:
            neighbor_node = neighbors[neighbor][0]
            new_cost = neighbors[neighbor][1]
            if neighbor_node in visited:
                continue
            visited.append(neighbor_node)

            child_loc = neighbor_node
            child_cost = cost + new_cost
            
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child, visited))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child, visited))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']

    print("h_values from compute heuristics: ")
    print(h_values)
    print()
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: 
    # first dict is for vertex constraints, second dict is for edge constraints
    # format is table[t] = list(constraint), where t is the timestep.

    table = {} 
    for i in range(len(constraints)):
        x = constraints[i]
        if x['agent'] == agent: #add constraint
            if table.get(x['timestep']) == None: table[x['timestep']] = list()
            table[x['timestep']].append(x)
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    #given a current node, return the path it took to reach it
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: 
    """
    Constraint table is a dictionary with timestep as key.
    First find the constraints for current time (if any),
    then iterate through each constraint.
    vertex/edge type constraint is determined by loc's length, and
    positive/negative attribute is added for 4.1.
    If True is returned, a constraint was violated and this path is discarded.
    """
    if constraint_table == None: return False #no constraint table

    constraints = constraint_table.get(next_time)
    if constraints == None: return False #no constraints at this time

    for i in range(len(constraints)):
        c = constraints[i]
        if len(c['loc']) == 1: #vertex constraint
            if (c['positive'] == True) and (next_loc != c['loc'][0]): return True #pos constraint
            elif next_loc == c['loc'][0]: return True #neg constraint
        else: #edge constraint
            if (c['positive'] == True) and (next_loc != c['loc'][1] or curr_loc != c['loc'][0]): return True #pos constraint
            elif next_loc == c['loc'][1] and curr_loc == c['loc'][0]: return True #neg constraint
    
    return False
    

def determine_earliest_goal(constraint_table):
    # finds the latest time for any constraint in the table, path must be at least this long
    if constraint_table == None: return 0 #no constraints
    c = list(constraint_table.keys())
    if len(c) == 0: return 0
    return max(c)

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def get_earliest_goal_timestep(agent, goal_loc, constraints):
    max_timestep = 0
    for constraint in constraints:
        if constraint['agent'] == agent:
            constraint_next_loc = constraint['loc'][0]
            if len(constraint['loc']) > 1:
                constraint_next_loc = constraint['loc'][1]
            
            if constraint_next_loc == goal_loc:
                max_timestep = max(max_timestep, constraint['timestep'] + 1)
    
    return max_timestep

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    print("Single Agent Planner: a_start")

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    constraint_table = build_constraint_table(constraints, agent)

    open_list = []
    closed_list = dict()

    earliest_goal_timestep = get_earliest_goal_timestep(agent, goal_loc, constraints)

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0, 'cost': 0}
    push_node(open_list, root)

    closed_list[(root['loc'], root['time'])] = root
    time = 0

    while len(open_list) > 0:
        curr = pop_node(open_list)
        curr_node = curr['loc']
        
        # update earliest_goal_timestep

        # Task 1.4: Added extra constraint to make sure all constraints listed are fufilled
        if curr_node == goal_loc and curr['time'] >= earliest_goal_timestep:
            return get_path(curr)

        neighbors = curr_node.edges
        for neighbor in neighbors:
            if neighbor is None:
                continue
            child_node = neighbors[neighbor][0]
            new_cost = curr_node.get_cost(child_node)[1]

            if is_constrained(curr['loc'], child_node, curr['time'] + 1, constraint_table):
                continue

            child = {'loc': child_node,
                    'g_val': curr['g_val'] + new_cost,
                    'h_val': h_values[child_node],
                    'parent': curr,
                    'time': curr['time'] + 1}

            if (child['loc'], child['time']) in closed_list:
                existing_node = closed_list[(child['loc'],child['time'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['time'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['time'])] = child
                push_node(open_list, child)
                
    return None  # Failed to find solutions
