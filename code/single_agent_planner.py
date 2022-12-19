import heapq
from graph import *

def get_sum_of_cost(paths): #returns sum of the cost of paths
    rst = 0
    for path in paths:
        for k in range(len(path)-1):
            if path[k].ID == path[k+1].ID: #same node, wait
                rst += 1
            else:
                rst += path[k].edges[path[k+1].ID][1] #edge cost
    return rst

def print_paths(paths): #prints out the paths in a more understandable way
    for path in range(len(paths)):
        path_str = ""
        for node in range(len(paths[path])):
            path_str += str(paths[path][node].ID)
            path_str += " "
        print("Agent " + str(path) + ": ", path_str)

def get_path_table(path):
    cost_table = {}
    node_table = {}
    for step in range(len(path)):
        if step == 0:
            cost = 0
            new_cost = 0
            prev =  Node(-1)  # dummy 
            curr = path[step]
        else:
            curr = path[step]
            if prev.ID == curr.ID: #same node, wait
                new_cost = cost + 1
            else:
                new_cost = cost + prev.edges[curr.ID][1] #edge cost

        cost_table[new_cost] = {'loc': curr, 'prev': prev, 'prev_cost': cost}
        node_table[curr] = {'cost': new_cost, 'prev': prev, 'prev_cost': cost}
        
        prev = curr
        cost = new_cost

    return cost_table, node_table

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
        neighbors[curr_node.ID] = [curr_node, 1]    # add wait cost option (may not be necessary here)

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
                    heapq.heappush(open_list, (child_cost, child_loc, child, visited))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child, visited))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']

    return h_values


def build_constraint_table(constraints, agent):
    table = {} 
    for i in range(len(constraints)):
        constraint = constraints[i]
        if constraint['agent'] == agent: #add constraint
            time = constraint['timestep']

            if type(time) == list:
                time = time[1]

            if time not in table:
                table[time] = []
            table[time].append(constraint)

    return table


def get_location(path, time):
    cost_table, _ = get_path_table(path)
    max_time = list(cost_table.keys())[-1]

    if time < 0:
        return path[0]
    elif time < max_time:
        loc = cost_table[time]['loc']
        if type(loc) == list:
            loc = loc[1]
        return loc
    else:
        loc = cost_table[max_time]['loc']
        if type(loc) == list:
            loc = loc[1]
        return loc

def get_prev_location(path, time):
    cost_table, node_table = get_path_table(path)
    max_time = list(cost_table.keys())[-1]
    
    if time < 0:
        return path[0]
    elif time < max_time:
        loc = cost_table[time]['loc']
        if type(loc) == list:
            loc = loc[1]
        return node_table[loc]['prev']
    else:
        loc = cost_table[max_time]['loc']
        if type(loc) == list:
            loc = loc[1]
        return node_table[loc]['prev']


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
    #print("is_constrained ? \n")
    # {6: [{'agent': 24, 'loc':Node, 'timestep': 6, 'positive': False}

    if constraint_table == {}:
        #print("No table")
        return False

    #print("constraint_table")
    #print(constraint_table)

    prev_time = 0
    for time, constraints in constraint_table.items():
        if time == next_time:
            for constraint in constraints:
                loc = constraint['loc']
                if type(loc) == list:
                    loc = loc[1]
                if loc == next_loc:
                    #print("is_constrained: Vertext Constraint found")
                    return True
        else:
            for constraint in constraints:
                constraint_time = constraint['timestep']
                if type(constraint_time) == list:
                    constraint_prev_loc = constraint['loc'][0]
                    constraint_next_loc = constraint['loc'][1]
                    if time >= constraint_time[0] and time <= constraint_time[1] and curr_loc == constraint_next_loc and next_loc == constraint_prev_loc:
                        # Edge
                        return True
                    if prev_time >= constraint_time[0] and prev_time <= constraint_time[1] and curr_loc == constraint_next_loc and next_loc == constraint_prev_loc:
                        # Edge
                        return True
        prev_time = time

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
            constraint_next_loc = constraint['loc']
            if type(constraint_next_loc) == list:
                constraint_next_loc = constraint_next_loc[1]
            
            time = constraint['timestep']
            if constraint_next_loc == goal_loc:
                if type(time) == list:
                    time = time[1]
                max_timestep = max(max_timestep, time + 1)

    return max_timestep

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    #print("Single Agent Planner: a_start")

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    #print("\nconstraints")
    #print(constraints)

    constraint_table = build_constraint_table(constraints, agent)
    #print("\nconstraint_table")
    #print(constraint_table)

    open_list = []
    closed_list = dict()

    earliest_goal_timestep = get_earliest_goal_timestep(agent, goal_loc, constraints)

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0}
    push_node(open_list, root)

    closed_list[(root['loc'],root['time'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        curr_node = curr['loc']
        
        if curr_node == goal_loc and curr['time'] >= earliest_goal_timestep:
            return get_path(curr)

        neighbors = curr_node.edges
        neighbors[curr_node.ID] = [curr_node, 1]    # add wait cost option

        for neighbor in neighbors:
            if neighbor is None:
                continue
            child_node = neighbors[neighbor][0]
            new_cost = curr_node.get_cost(child_node)[1]
            child_cost = curr['g_val'] + new_cost

            if is_constrained(curr['loc'], child_node, child_cost, constraint_table):
                continue

            child = {'loc': child_node,
                    'g_val': child_cost,
                    'h_val': h_values[child_node],
                    'parent': curr,
                    'time': curr['time'] + new_cost}

            if (child['loc'],child['time']) in closed_list:
                existing_node = closed_list[(child['loc'],child['time'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['time'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'],child['time'])] = child
                push_node(open_list, child)
                
    return None  # Failed to find solutions
