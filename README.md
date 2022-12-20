# MAPF with Weighted Paths

In CMPT 417, we explored the topic of multi-agent pathfinding in a two dimensional 4-neighbor grid, using time steps as the cost. This research alters that problem space by using nodes as locations and edges with weights for solution paths. These weighted paths produce the costs and the conflicts for the agents that are taking the paths. Algorithms that are being tested and compared to the results of the original problem are prioritized planning, Conflict-based Search (CBS), and Large Neighborhood Search (LNS).

## Set up

Note: below assumes that Git and Python 3.5+ are installed on your machine.


### Running tests
Clone the repository and navigate to the code folder. From there use the commands below to run each algorithm:

Prioritized
`python3 run_experiments.py --instance "testGraphs/graph*" --solver Prioritized`

CBS
`python3 run_experiments.py --instance "testGraphs/graph*" --solver CBS`

LNS
`python3 run_experiments.py --instance "testGraphs/graph*" --solver LSN`

### Formatting results from tests

Further instructions can be found in [the result interpreter file itself.](https://github.com/ohyeree1/417_project/blob/main/result_interpreter.py)
`python3 result_interpreter.py`
