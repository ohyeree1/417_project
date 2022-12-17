import time as timer
import heapq
from random import randint #randint(0,1) for 4.2
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from graph import *


def overlap(curr_time, constraint_time):
    a, b = curr_time
    ca, cb = constraint_time
    if (a <= ca and b >= ca) or (a >= ca and cb >= a):
        return True
    return False
    

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    path_table_1 = {}
    for step in range(len(path1)):
        if step == 0:
            cost = 0
            new_cost = 0
        else:
            if path1[step - 1].ID == path1[step].ID: #same node, wait
                new_cost = cost + 1
            else:
                new_cost = cost + path1[step - 1].edges[path1[step].ID][1] #edge cost
            
            a = path1[step - 1].ID
            b = path1[step].ID
            if b not in path_table_1:
                path_table_1[b] = []
            path_table_1[b].append((a, [cost, new_cost]))
    
        if new_cost not in path_table_1:
            path_table_1[new_cost] = [] # agent, loc, time_cost
        path_table_1[new_cost].append(path1[step]) # agent, loc, time_cost

    path_table_2 = {}
    for step in range(len(path2)):
        if step == 0:
            cost = 0
            new_cost = 0
        else:
            if path2[step - 1].ID == path2[step].ID: #same node, wait
                new_cost = cost + 1
            else:
                new_cost = cost + path2[step - 1].edges[path2[step].ID][1] #edge cost
            
        if new_cost not in path2:
            path_table_2[new_cost] = [] # agent, loc, time_cost
        path_table_2[new_cost].append(path2[step]) # agent, loc, time_cost

    # Vertex Collision
    for time_cost, loc in enumerate(path_table_2):
            print("time_cost, loc: ", time_cost, loc)
            if time_cost in path_table_1:
                if loc == path_table_1[time_cost]:
                    return [loc, time_cost]
    
    # Edge Collision
    for step in range(len(path_table_2)):
        if step == 0:
            cost = 0
            new_cost = 0
        else:
            if path2[step - 1].ID == path2[step].ID: #same node, wait
                new_cost = cost + 1
            else:
                curr_a = path2[step - 1]
                curr_b = path2[step]
                new_cost = cost + curr_a.edges[curr_b.ID][1] #edge cost

                print("curr_a: ", curr_a)
                if curr_a.ID in path_table_1:
                    for constraint in path_table_1[curr_a.ID]:
                        print("constraint: ", constraint)
                        if len(constraint) > 1 and constraint[0] == curr_b.ID and overlap(constraint[1], [cost, new_cost]):
                            return [(curr_a, curr_b), new_cost]

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
    

#insert paths_violate_constraint() here

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

def checkIfNew(x,constraints): #check if x is not in list of constraints yet
    for i in range(len(constraints)):
        if x == constraints[i]: return False
    return True

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

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
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
        self.push_node(root)

        """
        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))
        """
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit (done using clone(ar))
        constraintSplit = None

        while True:
            n = self.pop_node()
            if n == None: 
                raise BaseException('No solutions') #no solution found
                
            if len(n['collisions']) == 0: #solution found 
                self.print_results(n)
                self.runtime = timer.time() - self.start_time
                return n['paths']

            if disjoint: #disjoint splitting case
                constraintSplit = disjoint_splitting(n['collisions'][0])
                for j in range(2):
                    newConstraint = constraintSplit[j]
                    if checkIfNew(newConstraint,n['constraints']) == False: continue #duplicate

                    if newConstraint['positive'] == True: #positive case
                        q = {'cost': 0, 'constraints': clone(n['constraints']), 'paths': clone(n['paths']), 'collisions': []}
                        q['constraints'].append(newConstraint)
                
                        #positive constraint agents (everything violating the positive constraint needs to be adjusted)
                        pos_agents = paths_violate_constraint(newConstraint,q['paths'])
                        possible = True
                        for s in range(len(pos_agents)):
                            pAgent = pos_agents[s]
                            q['constraints'].append({'agent': pAgent, 'loc': newConstraint['loc'], 'timestep': newConstraint['timestep'], 'positive': False}) #implied negative constraint
                            q['paths'][pAgent] = a_star(self.my_map, self.starts[pAgent], self.goals[pAgent], self.heuristics[pAgent], pAgent, q['constraints'])
                            if q['paths'][pAgent] == None: #no path for a positive agent 
                                possible = False
                                break
                        if possible == False: continue #failed path was found

                        # q is a possible solution, add to list
                        q['collisions'] = detect_collisions(q['paths'])
                        q['cost'] = get_sum_of_cost(q['paths'])
                        self.push_node(q)
                
                    else: #negative case, same as normal CBS
                        q = {'cost': 0, 'constraints': clone(n['constraints']), 'paths': clone(n['paths']), 'collisions': []}
                        q['constraints'].append(newConstraint) 
                        #adjust agent's path
                        a = newConstraint['agent']
                        q['paths'][a] = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a], a, q['constraints'])
                        if q['paths'][a] != None:
                            q['collisions'] = detect_collisions(q['paths'])
                            q['cost'] = get_sum_of_cost(q['paths'])
                            self.push_node(q)
                     
            else: #normal splitting case 
                constraintSplit = standard_splitting(n['collisions'][0])
                for i in range(2):
                    newConstraint = constraintSplit[i]
                    if checkIfNew(newConstraint,n['constraints']): #not duplicate
                        q = {'cost': 0, 'constraints': clone(n['constraints']), 'paths': clone(n['paths']), 'collisions': []}
                        q['constraints'].append(newConstraint) 
                        #adjust agent's path
                        a = newConstraint['agent']
                        q['paths'][a] = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a], a, q['constraints'])
                        if q['paths'][a] != None:
                            q['collisions'] = detect_collisions(q['paths'])
                            q['cost'] = get_sum_of_cost(q['paths'])
                            self.push_node(q)
                            
    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
