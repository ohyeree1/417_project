import time as timer
from single_agent_planner import *
from graph import *

#template from prioritized.py, adjust as necessary
"""
test instructions:
python run_experiments.py --instance "testGraphs/graph*" --solver LNS

above tests CBS on the test graphs and writes to output to CBSresults.txt
swap CBS with other algorithms as needed
"""
class LargeNeighbourhoodSolver(object):
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

        # compute Dijkstra heuristics
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))

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
        General idea:
        1. create a solution (may have collisions or is suboptimal)
        2. destroy part of the solution and assume the rest is "optimal" for now
        3. fix the destroyed part of the solution
        4. if the repaired version results in a better solution (less collisions or faster), keep it
        5. repeat until solution is optimal or good enough
        """


        # 1. use single agent planner to create a solution
        # if 0 collisions, it's optimal

        for i in range(self.num_of_agents):
            result.append(a_star(self.my_map,self.starts[i],self.goals[i],self.heuristics[i],i,constraints))

        # 2. if there are collisions, choose the path stochastically with the most problems/or slowest and delete it
        # worst has 50%, 2nd worst 25%, 3rd worst 12.5%, so on until last two have equally small chance
        # For now assume the other paths are optimal


        # 3. Recreate the path with a stocastic search
        # Assign value to each action based on heuristic value
        # Causes collision: 0 (unless all options result in collision)
        # Optimal path according to heuristic has highest chance with suboptimal still possible since they may avoid collisions



        # 4. Reevaluate the solution, if better, keep it



        # 5. If no improvment is found after some number of attempts, return solution
        # Arbritarily choosing 5*# of agents
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
                    
                    format is [a,t], a is faulty area
                    if len(a) == 1, vertex constraint, else edge constraint
                    (the length part is already figured out by create_constraint_table)
                    
                    constraints.append({'agent': i, 'loc': collision_detected[0], 'timestep': collision_detected[1], 'positive': 0})

            # no collision found, add to result
            result.append(path)
            longestPath = max(len(path),longestPath)
            # debug for 2.4
            print("added path for Agent",i)
            print(path)


            
        """
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        #print("Paths in the solution:")
        #print_paths(result)
        #for j in range(self.num_of_agents):
            #print("Agent #"+str(j),result[j])
        #print(result)
        return result
