#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from large_neighbourhood import LargeNeighbourhoodSolver
from graph import *
#from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map):
    print("Node count: ", my_map.nodeCount)
    print("Agent count: ", my_map.agentCount)
    print("Node List")
    print(my_map.nodeList)
    for node in my_map.nodeList:
        if node is not None:
            print(node)

    for agent in range(len(my_map.agents)):
        print("Agent ", agent, ": start: ", my_map.agents[agent][0], ": goal: ", my_map.agents[agent][1])


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(fileName):
    f = Path(fileName)
    if not f.is_file():
        raise BaseException(fileName + " does not exist.")
    f = open(fileName,"r")
    line = f.readline()
    nodes,edges,agents = map(int,line.split(" "))
    g = Graph(nodes,agents)

    
    #add edges
    for i in range(edges):
        line = f.readline()
        a,b,c = map(int,line.split(" "))
        g.addEdge(a,b,c)

    #adjust agents
    for w in range(agents):
        line = f.readline()
        a,b = map(int,line.split(" "))
        g.agents[w][0] = a
        g.agents[w][1] = b

    f.close()
    return g

"""
test instructions:
python run_experiments.py --instance "testGraphs/graph*" --solver CBS

above tests CBS on the test graphs and writes to output to CBSresults.txt
swap CBS with other algorithms as needed
"""

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    
    output_file = str(args.solver)+"result.txt"
    result_file = open(output_file, "w", buffering=1)
    
    print_problem = False #set to False for full testing, printing out each map takes up a lot of space

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map = import_mapf_instance(file)
        if print_problem:
            print(my_map)
            print_mapf_instance(my_map)

        if args.solver == "CBS":
            print("***Run CBS***")
            cbs = CBSSolver(my_map)
            paths = cbs.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map)
            paths = solver.find_solution()
        elif args.solver == "LNS":
            print("***Run LNS***")
            lns = LargeNeighbourhoodSolver(my_map)
            paths = lns.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))

    result_file.close()
    

    # result writer with additional details, used for generating 3.4 and 4.3 detailed results
    """
    result_file = open("extendedResults4_3.txt", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)
        cbs = None

        if args.solver == "CBS":
            print("***Run CBS***")
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")
        
        cost = get_sum_of_cost(paths)
        result_file.write("{} \t {} \t {} \t {} ".format(file, cost, cbs.runtime, cbs.num_of_generated))
        result_file.write("(")
        result_file.write("{}".format(cbs.num_of_expanded))
        result_file.write(")\n")
        


        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
    """
    