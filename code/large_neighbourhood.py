import time as timer
from single_agent_planner import *
from graph import *
from LNSHelper import *
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

    def count_collisions(self, agent1, agent2): #return all collisions between two agents given their location tables
        collisions = 0
        end_time = max(agent1[1],agent2[1])
        #collisions happening directly on a timestep 
        for i in range(end_time):
            a = self.get_location(agent1,i)
            b = self.get_location(agent2,i)
            if a == b and len(a) == 1 and len(b) == 1: #vertex collision
                collisions += 1
            elif len(a) == 2 and len(b) == 2: #possible edge traversal collision
                if a[0] == b[1] and b[0] == a[1]: #edge traversal collision
                    collisions += 1 
                    """
                    side effect is that for long edges the collision 
                    may count multiple times, but does not affect the 
                    goal of having no collisions in the final solution
                    """
            #if lengths differ then no collision as one is one a vertex
            #and the other is traversing an edge

        #collisions happening between two timesteps
        #only possible case is when two agents swap nodes
        #via an edge of cost 1, higher cost is caught by direct timestep

        for j in range(end_time-1):
            a = self.get_location(agent1,j)
            aa = self.get_location(agent1,j+1)
            b = self.get_location(agent2,j)
            bb = self.get_location(agent2,j+1)
            if max(len(a),len(b),len(aa),len(bb)) == 1 and a == bb and b == aa: #edge 1 collision
                collisions += 1

        return collisions

    def get_location(self,table,x): #return location at time x given table
        if x >= table[1]: return table[0][table[1]] #agent has stopped moving
        else: return table[0][x]

    def create_location_table(self,path): #return a dict for where an agent is based on "time"
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
                    ans[j] = [path[i].ID,path[i+1].ID] #in process of traversing two nodes
                time += edge_cost
        ans[time] = [path[-1].ID] #last node
        return [ans,time] #after time agent will be at last node

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        # 1. use single agent planner to create a solution
        # if 0 collisions, it's optimal

        for i in range(self.num_of_agents):
            result.append(a_star(self.my_map,self.starts[i],self.goals[i],self.heuristics[i],i,[]))

        # 1.5. Check if there are collisions, if there aren't any, then this setup is guaranteed to be optimal
        
        collisions = 0 #tracks TOTAL number of collisions
        collision_pair_freq = list() #tracks number of collisions between a pair of agents
        collision_agent_freq = [0]*self.num_of_agents #tracks number of collisions an agent is involved in
        for t in range(self.num_of_agents):
            collision_pair_freq.append([0]*self.num_of_agents)
        agent_location = [{}]*self.num_of_agents
        for j in range(self.num_of_agents):
            agent_location[j] = self.create_location_table(result[j])

        #determine each collision occuring
        for m in range(self.num_of_agents):
            for n in range(m+1,self.num_of_agents):
                pair_collisions = self.find_collisions(agent_location[m],agent_location[n])
                collision_pair_freq[m][n] += pair_collisions
                collision_pair_freq[n][m] += pair_collisions
                collision_agent_freq[n] += pair_collisions
                collision_agent_freq[m] += pair_collisions
                collisions += pair_collisions
                
        if collisions == 0: #shortest path for each agent is optimal
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

        #independent a* has problems, use stocastic search to fix
        #find costs of each path first
        result_costs = list()
        for pa in range(self.num_of_agents):
            result_costs.append(cost_of_path(result[pa]))
        


        # 2. Choose the path stochastically with the most problems/or slowest and delete it
        # worst has 50%, 2nd worst 25%, 3rd worst 12.5%, so on until last two have equally small chance
        # For now assume the other paths are optimal
        missCount = 0
        while missCount < 5*self.num_of_agents: #stop stocastic search after certain number of attempts without improvement
            path_ratings = list()
            for q in range(self.num_of_agents):
                path_ratings.append([collision_agent_freq(q),result_costs[q],q])
        
        # order the paths from best to worst (via collisions involved in, then path cost), then pseudorandomly select one to fix
            path_ratings.sort()
            agent_to_fix = path_ratings[weighted_random(self.num_of_agents,0.3)][2] #fix result[agent_to_fix]
        

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
