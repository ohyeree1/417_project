import time as timer
import heapq
from random import randint #randint(0,1) for 4.2
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, print_path, print_paths, get_path_table, get_location
from graph import *
import copy
import sys

sys.setrecursionlimit(3000)

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

    cost_table_1, node_table_1, _, _ = get_path_table(path1)
    cost_table_2, node_table_2, _, _ = get_path_table(path2)

    # Check Vertex Collision
    for time_cost, value_1 in cost_table_1.items():
        if time_cost in cost_table_2:
            value_2 = cost_table_2[time_cost]
            
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
    
    loc = collision['loc']
    if type(loc) == list:
        print("Standard splitting for Edge Collision")
        return [{'agent': collision['a1'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive': False},
                {'agent': collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive': False}]
    else:
        print("Standard splitting for Vertex Collision")
        return [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False},
                {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}]  


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
    if r == 0: a, b = False, True

    loc = collision['loc']
    if type(loc) == list:
        print("Disjoint splitting for Edge Collision")
        return [{'agent': collision['a1'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive': a},
                {'agent': collision['a2'], 'loc': [collision['loc'][0],collision['loc'][1]], 'timestep': collision['timestep'], 'positive': b},
                {'agent': collision['a2'], 'loc': [collision['loc'][1],collision['loc'][0]], 'timestep': collision['timestep'], 'positive': b}]
    else:
        print("Disjoint splitting for Vertexß Collision")
        return [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': a},
                {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': b}]


def paths_violate_constraint(constraint, paths):
    print("------------------------------------- paths_violate_constraint ---------------------------------")
    print("constraint:")
    print(constraint)
    print()
    assert constraint['positive'] is True
    result = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue

        time = constraint['timestep']
        if type(time) == list:
            time = time[1]

        print("time: ", time)
        loc_info = get_location(paths[i], time)
        if loc_info == {}:
            continue    # no collision found

        prev = loc_info['prev']
        curr = loc_info['loc']
        prev_time = loc_info['prev_cost']
        curr_time = loc_info['cost']
        print("prev, curr, prev_time, curr_time")
        print(prev.ID, curr.ID, prev_time, curr_time)
        
        if time == 0 or curr == None:
            continue

        result.append(i)
        loc = constraint['loc']
        if type(loc) == Node:
            if loc == curr:   # Vertext Constraint
                result.append(i)
        else:  # Edge Constraint
            print("paths_violate_constraint: Edge constraint found")
            if [prev, curr] == loc or [curr, prev] == loc:
                print("Checking constraint edge locations")
                print(loc[0].ID, loc[1].ID)
                result.append(i)

    print("paths_violate_constraint, result: ", result)
    print()
    return result

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
                return []
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        while len(self.open_list) > 0:
            node = self.pop_node()
            if node is None:
                return [] # No Solution

            if len(node['collisions']) == 0:
                self.print_results(node)
                self.runtime = timer.time() - self.start_time
                return node['paths']
            
            collision = node['collisions'][0]
            
            if disjoint is True:
                constraints = disjoint_splitting(collision)
                for constraint in constraints:
                    if constraint in node['constraints']:
                        continue    # dup

                    child_node = copy.deepcopy(node)
                    child_node['constraints'].append(constraint)

                    if constraint['positive']:
                        print("Disjoint with positive constraint")
                        negative_agents = paths_violate_constraint(constraint, child_node['paths'])
                        for negative_agent in negative_agents:
                            new_constraint = {
                                'agent': negative_agent,
                                'loc': constraint['loc'],
                                'timestep': constraint['timestep'],
                                'positive': False,
                            }
                            if new_constraint not in child_node['constraints']:
                                child_node['constraints'].append(new_constraint)
                        possible = True

                        for negative_agent in negative_agents:
                            path = a_star(self.my_map, self.starts[negative_agent], self.goals[negative_agent], self.heuristics[negative_agent], negative_agent, child_node['constraints'])
                            if path:
                                child_node['paths'][negative_agent] = path
                            else:
                                possible = False
                                break
                        else:
                            child_node['collisions'] = detect_collisions(child_node['paths'])
                            child_node['cost'] = get_sum_of_cost(child_node['paths'])
                            self.push_node(child_node)
                        if not possible:
                            continue

                    else:
                        # Disjoint but negative constraint
                        print("Disjoint with negative constraint")
                        agent = constraint['agent']
                        path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, child_node['constraints'])
                        if path is not None:
                            child_node['paths'][agent] = path

                            child_node['collisions'] = detect_collisions(child_node['paths'])
                            child_node['cost'] = get_sum_of_cost(child_node['paths'])
                            self.push_node(child_node)


#             #
#             for constraint in constraints:
#                 if constraint in node['constraints']:
#                     continue
                
#                 child_node = copy.deepcopy(node)
#                 child_node['constraints'].append(constraint)
                
#                 # deal with positive constraints
#                 if disjoint and constraint['positive']:
#                     print("Using Disjoint Splitting\n")

#                     negative_agents = paths_violate_constraint(constraint, node['paths'])
#                     for negative_agent in negative_agents:
#                         new_constraint = {
#                             'agent': negative_agent,
#                             'loc': constraint['loc'],
#                             'timestep': constraint['timestep'],
#                             'positive': False,
#                         }
#                         if new_constraint not in child_node['constraints']:
#                             child_node['constraints'].append(new_constraint)

#                     for negative_agent in negative_agents:
#                         path = a_star(self.my_map, self.starts[negative_agent], self.goals[negative_agent], self.heuristics[negative_agent], negative_agent, child_node['constraints'])
#                         if path:
#                             child_node['paths'][negative_agent] = path
#                         else:
#                             break
#                     else:
#                         child_node['collisions'] = detect_collisions(child_node['paths'])
#                         child_node['cost'] = get_sum_of_cost(child_node['paths'])
#                         self.push_node(child_node)

            else:   # Standard Splitting
                constraints = standard_splitting(collision)
                for constraint in constraints:
                    if constraint in node['constraints']:
                        continue    # dup

                    child_node = copy.deepcopy(node)
                    child_node['constraints'].append(constraint)

                    agent = constraint['agent']
                    print("\nUsing Standard Splitting")
                    path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, child_node['constraints'])

                    if path is not None:
                        print("new path found:")
                        print_path(path)
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
