import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_path_table
from cbs import detect_collision
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
        
    def find_collision(self, path, result):
        for prev_path in result:
            collision = detect_collision(path, prev_path)
            if collision != None:
                return collision

<<<<<<< HEAD
        #print("probs here")
        curr_timetable = self.build_timetable_for_path(path)

        for time_cost, loc in enumerate(curr_timetable):
            #print("time_cost, loc: ", time_cost, loc)
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

                    #debug for constraints
                    #print("curr_a: ", curr_a)
                    #print(edge_constraints)
                    if curr_a.ID in edge_constraints:
                        for constraint in edge_constraints[curr_a.ID]:
                            if constraint[0] == curr_b.ID and self.overlap(constraint[1], [cost, new_cost]):
                                return [(curr_a, curr_b), new_cost]

        return None #no collisions found
=======
        return None
>>>>>>> main

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        for i in range(self.num_of_agents):  # Find path for each agent
            path = None
            while path == None:

                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
                print("path: ", path)
                if path is None:
                    return []   # BaseException('No solutions')

                collision = self.find_collision(path, result)
                if collision != None:
                    path = None
                    constraints.append(
                        {
                            'agent': i,
                            'loc': collision[0],
                            'timestep': collision[1],
                            'positive': False
                        }
                    )

            result.append(path)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print("Paths in the solution:")

<<<<<<< HEAD
        #print(result)
=======
>>>>>>> main
        for agent in range(self.num_of_agents):
            path_str = ""
            for node in result[agent]:
                path_str += str(node.ID)
                path_str += " "
            print("Agent " + str(agent) + ": ", path_str)

        return result
