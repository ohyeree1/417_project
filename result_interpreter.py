# helper file for interpreting the output results
# Assumes the file stores the results from full testing
from pathlib import Path

"""
file options:
Prioritized
CBSStandard
CBSDisjoint
LNS
"""
file = "LNS"
fileName = "results/"+file+".txt"
f = Path(fileName)
if not f.is_file():
    raise BaseException(fileName + " does not exist.")

f = open(fileName,"r")
results = [0]*80 # 0 to 79
for i in range(80): #splitting line to get integer values
    line = f.readline()
    a,b = line.split(",")
    cost = int(b)
    c,dummy = a.split(".")
    dummy,e = c.split("phs/graph")
    results[int(e)-1] = cost

"""
test func to see if dict was set up properly
for j in range(80):
    print(j,results[j])
"""
output_file = "results/"+file+"formatted.txt"
output = open(output_file, "w", buffering=1)


"""
Graphs are numbered as following:
0-19 -> small graphs (2 agents, 10 nodes)
20-39 -> medium graphs (8 agents, 50 nodes)
40-59 -> large graphs (20 agents, 250 nodes)
Each group of 20 is split into 5 groups of 4
with varying edge densities (1.1,1.25,1.5,2,3)
ie. graphs 1-4 have density 1.1, 5-8 density 1.25 etc.
60-79 -> special graphs that require path of at least 8 values
60-69 -> small special graph (50 nodes, 2/5 agents)
70-79 -> large special graph (250 nodes, 12,25 agents)
all of these graphs use edge density 1.5
"""

# determine success rate
size_clear_rate = [0,0,0,0,0]
size_average = [0,0,0,0,0]
edge_clear_rate = [0,0,0,0,0] #not counting 60-79
total_clear = 0
for j in range(80):
    if results[j] != 0: #solution found
        total_clear += 1
        if j >= 60: #special graph
            if j >= 70:
                size_clear_rate[4] += 1
                size_average[4] += results[j]
            else:
                size_clear_rate[3] += 1
                size_average[3] += results[j]
        else: #normal graph
            if j < 20: #small
                size_clear_rate[0] += 1
                size_average[0] += results[j]
            elif j < 40: #medium
                size_clear_rate[1] += 1
                size_average[1] += results[j]
            else: #large
                size_clear_rate[2] += 1
                size_average[2] += results[j]
            edge_clear_rate[(j//4)%5] += 1

for snth in range(5):
    if size_clear_rate[snth] == 0: size_average[snth] = 0
    else: size_average[snth] = (size_average[snth]/size_clear_rate[snth])

output.write("N = node count, K = agent count, D = edge density\n")
output.write(file+" results:\n\n")
output.write("Clear rates:\n")
output.write(str(total_clear)+" of 80 total maps were passed\n")
output.write("N = 10, K = 2: "+str(size_clear_rate[0])+" of 20 cleared\n")
output.write("N = 50, K = 8: "+str(size_clear_rate[1])+" of 20 cleared\n")
output.write("N = 250, K = 20: "+str(size_clear_rate[2])+" of 20 cleared\n")
output.write("Alternate N = 50: "+str(size_clear_rate[3])+" of 10 cleared\n")
output.write("Alternate N = 250: "+str(size_clear_rate[4])+" of 10 cleared\n\n")
output.write("D = 1.1: "+str(edge_clear_rate[0])+" of 12 cleared\n")
output.write("D = 1.25: "+str(edge_clear_rate[1])+" of 12 cleared\n")
output.write("D = 1.5: "+str(edge_clear_rate[2])+" of 12 cleared\n")
output.write("D = 2: "+str(edge_clear_rate[3])+" of 12 cleared\n")
output.write("D = 3: "+str(edge_clear_rate[4])+" of 12 cleared\n\n")
output.write("Averages (only counts passed cases):\n")
output.write("N = 10, K = 2: "+str(size_average[0])+"\n")
output.write("N = 50, K = 8: "+str(size_average[1])+"\n")
output.write("N = 250, K = 20: "+str(size_average[2])+"\n")
output.write("Alternate N = 50: "+str(size_average[3])+"\n")
output.write("Alternate N = 250: "+str(size_average[4])+"\n\n")



output.close()



