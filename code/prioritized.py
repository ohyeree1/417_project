import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from graph import *

class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

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
            
        self.CPU_time = 0

        # compute heuristics for the low-level search (to be adjusted)
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))
        

    def build_timetable_and_edge_constraint_table(self, paths):
        if paths == []:
            return None, None
            
        table = {}
        edge_constraints = {}
        cost = 0
        new_cost = 0
        for saved_path in paths:                
            for step in range(len(saved_path)):
                if step == 0:
                    cost = 0

                else:
                    if saved_path[step - 1].ID == saved_path[step].ID: #same node, wait
                        new_cost = cost + 1
                    else:
                        new_cost = cost + saved_path[step - 1].edges[saved_path[step].ID][1] #edge cost
                    
                    a = saved_path[step - 1].ID
                    b = saved_path[step].ID
                    if b not in edge_constraints:
                        edge_constraints[b] = []
                    edge_constraints[b].append((a, [cost, new_cost]))
    
                if new_cost not in table:
                    table[new_cost] = [] # agent, loc, time_cost
                # table[cost].add((agent, saved_path[step])) # agent, loc, time_cost
                table[new_cost].append(saved_path[step]) # agent, loc, time_cost
                
        return table, edge_constraints

    def build_timetable_for_path(self, path):
        table = {}
        for step in range(len(path)):
            if step == 0:
                cost = 0
                new_cost = 0
            else:
                if path[step - 1].ID == path[step].ID: #same node, wait
                    new_cost = cost + 1
                else:
                    new_cost = cost + path[step - 1].edges[path[step].ID][1] #edge cost
                
            if new_cost not in table:
                table[new_cost] = [] # agent, loc, time_cost
            table[new_cost].append(path[step]) # agent, loc, time_cost

        return table

    def overlap(self, curr_time, constraint_time):
        a, b = curr_time
        ca, cb = constraint_time

        if (a <= ca and b >= ca) or (a >= ca and cb >= a):
            return True

        return False

    def find_collision(self, path, result):
        # We know that the paths of the previous agents do not collide
        timetable, edge_constraints = self.build_timetable_and_edge_constraint_table(result)
        if timetable is None:
            return None

        print("probs here")
        curr_timetable = self.build_timetable_for_path(path)

        for time_cost, loc in enumerate(curr_timetable):
            print("time_cost, loc: ", time_cost, loc)
            if time_cost in timetable:
                # Edge Collision
                if loc == timetable[time_cost]:
                    return [loc, time_cost]
            
        for step in range(len(path)):
            if step == 0:
                cost = 0
            else:
                if path[step - 1].ID == path[step].ID: #same node, wait
                    new_cost = cost + 1
                else:
                    curr_a = path[step - 1]
                    curr_b = path[step]
                    new_cost = cost + curr_a.edges[curr_b.ID][1] #edge cost

                    print("curr_a: ", curr_a)
                    print(edge_constraints)
                    if curr_a.ID in edge_constraints:
                        for constraint in edge_constraints[curr_a.ID]:
                            if constraint[0] == curr_b.ID and self.overlap(constraint[1], [cost, new_cost]):
                                return [(curr_a, curr_b), new_cost]

        return None #no collisions found

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        """
        {'agent': 0, 'loc': [(1,5)], 'timestep': 4}          1.2 constraint
        {'agent': 1, 'loc': [(1,2), (1,3)], 'timestep': 1}   1.3 constraint
        {'agent': 0, 'loc': [(1,5)], 'timestep': 10}         1.4 constraint

        1.5 constraints
        {'agent': 1, 'loc': [(1,4)], 'timestep': 3},
        {'agent': 1, 'loc': [(1,4), (1,3)], 'timestep': 3},
        {'agent': 1, 'loc': [(1,3)], 'timestep': 2},
        {'agent': 1, 'loc': [(1,4)], 'timestep': 2},
        {'agent': 1, 'loc': [(1,2)], 'timestep': 2}
        """

        for i in range(self.num_of_agents):  # Find path for each agent
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            ##############################

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            if path is None:
                raise BaseException('No solutions')
            
            collision_detected = self.find_collision(path, result)
            if collision_detected is not None: 
                #collision found, add constraint and try again
                """
                format is [a,t], a is faulty area
                if len(a) == 1, vertex constraint, else edge constraint
                (the length part is already figured out by create_constraint_table)
                """
                constraints.append({'agent': i, 'loc': collision_detected[0], 'timestep': collision_detected[1], 'positive': 0})

            # no collision found, add to result
            result.append(path)


        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print("Paths in the solution:")

        print(result)
        for agent in range(self.num_of_agents):
            path_str = ""
            for node in result[agent]:
                path_str += str(node.ID)
                path_str += " "
            print("Agent " + str(agent) + ": ", path_str)


        return result
