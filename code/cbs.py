import time as timer
import heapq
from random import randint #randint(0,1) for 4.2
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, print_paths, get_path_table, get_prev_location, get_location
from graph import *
import copy


def overlap(curr_time, constraint_time):
    a, b = curr_time
    ca, cb = constraint_time
    return max(0, min(b, cb) - max(a, ca)) > 0


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    path_table_1, node_table_1 = get_path_table(path1)
    path_table_2, node_table_2 = get_path_table(path2)

    print("path_table_1")   # {'loc': curr, 'prev': prev, 'prev_cost': cost}
    print(path_table_1)
    print("node_table_1")
    print(node_table_1)
    print("path_table_2")
    print(path_table_2)
    print("node_table_2")
    print(node_table_2)

    # Check Vertex Collision
    for time_cost, value_1 in path_table_1.items():
        print("time_cost, value: ", time_cost, value_1)
        if time_cost in path_table_2:
            value_2 = path_table_2[time_cost]
            
            if value_1['loc'] == value_2['loc']:
                return [value_1['loc'], time_cost]
    
    # Check Edge Collision
    for node, value_1 in node_table_1.items():
        prev_1 = value_1['prev']
        if node in node_table_2 and prev_1 in node_table_2 and node_table_2[prev_1]['prev'] == node:
            # There may be an edge collision
            time_interval_1 = [value_1['prev_cost'], value_1['cost']]
            time_interval_2 = [node_table_2[prev_1]['prev_cost'], node_table_2[prev_1]["cost"]]
            if overlap(time_interval_1, time_interval_2):
                print("Edge Collision")
                return [[prev_1, node], [value_1['prev_cost'], value_1['cost']]]

    return None #no collisions found


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)-1):
        for j in range(i+1,len(paths)):
            collision = detect_collision(paths[i],paths[j])
            if collision:
                loc, cost = collision
                collisions.append({'a1': i, 'a2': j, 'loc': loc, 'timestep': cost})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    
    if len(collision['loc']) == 1: #vertex collision
        return [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False},
                {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}]
    else: #edge collision
        return [{'agent': collision['a1'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive': False},
                {'agent': collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False}]
    


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    # 0 = False, 1 = True
    a,b = True,False
    r = randint(0,1)
    if r == 0: a,b = False,True

    if len(collision['loc']) == 1: #vertex collision
        return [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': a},
                {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': b}]

    else: #edge collision
        return [{'agent': collision['a1'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive': a},
                {'agent': collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive': b}]
    
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    result = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue        
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_prev_location(paths[i], constraint['timestep'])

        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                result.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                result.append(i)

    print("paths_violate_constraint, result: ", result)
    print()
    return result

# def checkIfNew(x,constraints): #check if x is not in list of constraints yet
#     for i in range(len(constraints)):
#         if x == constraints[i]: return False
#     return True

def clone(ar): #creates a deep copy of an array
    new_ar = list()
    for i in range(len(ar)):
        new_ar.append(ar[i])
    return new_ar


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, graph):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = graph.nodeList
        self.starts = list()
        self.goals = list()
        self.num_of_agents = graph.agentCount
        for i in range(self.num_of_agents):
            self.starts.append(graph.nodeList[graph.agents[i][0]])
            self.goals.append(graph.nodeList[graph.agents[i][1]])


        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.runtime = 0

        self.open_list = []

        # compute heuristics for the low-level search (to be adjusted)
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))


    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        try:
            _, _, id, node = heapq.heappop(self.open_list)
            #print("Expand node {}".format(id))
            self.num_of_expanded += 1
            return node
        except IndexError: return None #empty heap

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
    
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        print("collisions: ", root['collisions'])

        self.push_node(root)

        while len(self.open_list) > 0 and len(self.open_list) < 1000:
            node = self.pop_node()

            if len(node['collisions']) == 0:
                self.print_results(node)
                return node['paths']
            
            collision = node['collisions'][0]
        
        if disjoint:
            constraints = disjoint_splitting(collision)
        else:
            constraints = standard_splitting(collision)
        
        for constraint in constraints:
            child_node = copy.deepcopy(node)
            if constraint not in child_node['constraints']:
                child_node['constraints'].append(constraint)
            else:
                continue

            # deal with positive constraints
            if disjoint and constraint['positive']:
                print("Use Disjoint Splitting. constraint:")
                print(constraint)
                print()

                negative_agents = paths_violate_constraint(constraint, path['paths'])
                for negative_agent in negative_agents:
                    new_constraint = {
                        'agent': negative_agent,
                        'loc': constraint['loc'],
                        'timestep': constraint['timestep']
                    }
                    if new_constraint not in child_node['constraints']:
                        child_node['constraints'].append(new_constraint)

                for negative_agent in negative_agents:
                    path = a_star(self.my_map, self.starts[negative_agent], self.goals[negative_agent], self.heuristics[negative_agent], negative_agent, child_node['constraints'])
                    if path:
                        child_node['paths'][negative_agent] = path
                    else:
                        break
                else:
                    child_node['collisions'] = detect_collisions(child_node['paths'])
                    child_node['cost'] = get_sum_of_cost(child_node['paths'])
                    self.push_node(child_node)

            else:   # Standard Splitting
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, child_node['constraints'])

                if path:
                    child_node['paths'][agent] = path
                    child_node['collisions'] = detect_collisions(child_node['paths'])
                    child_node['cost'] = get_sum_of_cost(child_node['paths'])
                    self.push_node(child_node)
        
        print("No Solution")
        return []


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        print("path:")
        print_paths(node['paths'])
