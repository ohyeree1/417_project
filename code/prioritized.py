import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_collision(self, path, result):
        if result == []: return None #first path created, no possible collisions
        path_len = len(path)

        #test for vertex constraints (2.1)
        for i in range(len(result)):
            test_len = len(result[i])
            """
            search through the longer of the two paths,
            the shorter path's location after ending is
            just the last coordinate. This prevents
            agents from moving over those already finished.
            (2.3)
            """
            for j in range(max(path_len,test_len)): 
                if path[min(j,path_len-1)] == result[i][min(j,test_len-1)]: return [[path[min(j,path_len-1)]],j]
        
        #test for edge constraints (2.2)
        for k in range(len(result)):
            test_len = len(result[k])
            for l in range(min(path_len,test_len)-1): 
                if path[l] == result[k][l+1] and path[l+1] == result[k][l]: return [[path[l],path[l+1]],l+1]
        
        return None #no collisions found

    def countOpen(self, my_map): #count number of spots that are not walls (2.4)
        ans = 0
        for i in range(len(my_map)):
            for j in range(len(my_map[i])):
                if not my_map[i][j]: ans += 1
        return ans

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
        
        longestPath = 0 #for creating a time limit (2.4)
        openSpots = self.countOpen(self.my_map)

        for i in range(self.num_of_agents):  # Find path for each agent
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            ##############################
            path = None
            while path == None:
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints, openSpots+longestPath)
                if path is None:
                    raise BaseException('No solutions')
                collision_detected = self.find_collision(path,result)
                if collision_detected != None: 
                    #collision found, add constraint and try again
                    path = None
                    """
                    format is [a,t], a is faulty area
                    if len(a) == 1, vertex constraint, else edge constraint
                    (the length part is already figured out by create_constraint_table)
                    """
                    constraints.append({'agent': i, 'loc': collision_detected[0], 'timestep': collision_detected[1], 'positive': 0})

            # no collision found, add to result
            result.append(path)
            longestPath = max(len(path),longestPath)
            # debug for 2.4
            print("added path for Agent",i)
            print(path)


            

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print("Paths in the solution:")
        for j in range(self.num_of_agents):
            print("Agent #"+str(j),result[j])
        #print(result)
        return result
