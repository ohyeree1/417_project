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

other notes:
astar returns a list of nodes, not node ids (see use of get_sum_of_costs)
"""
class LargeNeighbourhoodSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, graph):
        self.my_map = graph.nodeList #graph object containing the map
        self.starts = list() #starting nodes for each agent in order
        self.goals = list() #ending nodes for each agent in order
        self.num_of_agents = graph.agentCount #number of agents
        for i in range(self.num_of_agents):
            self.starts.append(graph.nodeList[graph.agents[i][0]])
            self.goals.append(graph.nodeList[graph.agents[i][1]])
            
        self.CPU_time = 0

        # compute Dijkstra heuristics, array of dicts with each corresponding 
        # to the nth's agent goal being the "root" of the tree
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))

    

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
        collision_agent_freq = [0]*self.num_of_agents #tracks number of collisions an agent is involved in
        collision_pair_freq = list() #tracks number of collisions between a pair of agents
        for this_varibale_will_not_be_used in range(self.num_of_agents):
            collision_pair_freq.append([0]*self.num_of_agents)

        #create location tables for each agent
        agent_location = [{}]*self.num_of_agents
        for j in range(self.num_of_agents):
            agent_location[j] = create_location_table(result[j])

        #determine each collision occuring
        for m in range(self.num_of_agents):
            for n in range(m+1,self.num_of_agents):
                pair_collisions = count_collisions(agent_location[m],agent_location[n])
                collision_pair_freq[m][n] += pair_collisions
                collision_pair_freq[n][m] += pair_collisions
                collision_agent_freq[n] += pair_collisions
                collision_agent_freq[m] += pair_collisions
                collisions += pair_collisions
                
        if collisions == 0: #shortest path for each agent is optimal, can end early
            self.CPU_time = timer.time() - start_time

            print("\n Found a solution! \n")
            print("LNS ended early due to Indpendent being an optimal solution.")
            print("CPU time (s):    {:.2f}".format(self.CPU_time))
            print("Sum of costs:    {}".format(get_sum_of_cost(result)))
            """
            print("Paths in the solution:")
            print_paths(result)
            for j in range(self.num_of_agents):
                print("Agent #"+str(j),result[j])
            print(result)
            """
            return result

        #independent a* has problems, use stocastic search to fix
        #find costs of each path first
        result_costs = list()
        for pa in range(self.num_of_agents):
            result_costs.append(cost_of_path(result[pa]))
        


        # 2. Choose the path stochastically with the most problems/or slowest and delete it
        # worst has 50%, 2nd worst 25%, 3rd worst 12.5%, so on until last two have equally small chance
        # For now assume the other paths are optimal
        miss_count = 0
        while miss_count < 5*self.num_of_agents: #stop stocastic search after certain number of attempts without improvement
            path_ratings = list()
            for q in range(self.num_of_agents):
                path_ratings.append([collision_agent_freq[q],result_costs[q],q])
        
            # order the paths from best to worst (via collisions involved in, then path cost), then pseudorandomly select one to fix
                path_ratings.sort()
                agent_to_fix = path_ratings[weighted_random(self.num_of_agents,0.3)][2] #fix result[agent_to_fix]
        

            # 3. Recreate the path with a stocastic search
            # Assign value to each action based on heuristic value
            # Path chosen ALWAYS greedingly chooses action that minimizes collisions
            # A* value is used as tiebreaker, then order choices from best to worst
            # Optimal path according to heuristic has highest chance with suboptimal still possible since they may avoid collisions
            # Extra logic is used when waiting on goal node, prioritize waiting until a collision would occur, then move out
            # minimum time = last collision with other path

            atf = agent_to_fix #shorthand to make code easier to manage
            new_path = find_path(self.my_map,self.heuristics[self.goals[atf]],agent_location,self.starts[atf],self.goals[atf],atf)

            # 4. Reevaluate the solution, if better, keep it
            np_collisions = list()
            new_table = create_location_table(new_path)
            for y in range(self.num_of_agents):
                if y != agent_to_fix:
                    np_collisions.append(count_collisions(new_table,agent_location[y]))
                else:
                    new_path.collisions.append(0)
            np_cost = cost_of_path(new_path)
            np_collision_count = sum(np_collisions)
            if np_collision_count < collision_agent_freq[atf] or (np_collision_count == collision_agent_freq[atf] and np_cost < result_costs[atf]):
                #replace path with better one, reset missCount
                miss_count = 0

                #adjust collision trackers
                collisions -= (collision_agent_freq[atf]-np_collision_count)
                collision_agent_freq[atf] = np_collision_count
                for snth in range(self.num_of_agents):
                    collision_pair_freq[atf][snth] = np_collisions[snth]
                    collision_agent_freq[snth] -= (collision_pair_freq[snth][atf]-np_collisions[snth])
                    collision_pair_freq[snth][atf] = np_collisions[snth]

                #replace path and its cost
                result[atf] = new_path
                result_costs[atf] = cost_of_path(new_path)
                    

            else: #not better, increase missCount
                miss_count += 1


        # 5. If no improvment is found after some number of attempts, return solution
        # Arbritarily choosing 5*# of agents
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("Number of Collisions:   {}".format(collisions))
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        """
        print("Paths in the solution:")
        print_paths(result)
        for j in range(self.num_of_agents):
            print("Agent #"+str(j),result[j])
        print(result)
        """
        return result
