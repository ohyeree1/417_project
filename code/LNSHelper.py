#helper functions used in LNS search (stochastic variation)
from random import random as e

def weighted_random(choice_count: int, p: float) -> int:
    #given n choices labelled 0 to n-1, choose with a weighted distribution according to following:
    #chance of picking first value = p
    #chance of picking second value = p*(1-p)
    #chance of picking third value = p*(1-(p*1-p))
    #repeat until choice is found (or just give up and choose first one if no choice is found after arbritarily long)
    x = 0
    for i in range(choice_count*50):
        if e() < p: return x
        x = (x+1) % choice_count
    return 0


def cost_of_path(p): #return the cost of a single given path
    c = 0
    for k in range(len(p)-1):
        if p[k].ID == p[k+1].ID: #same node, wait
            c += 1
        else:
            c += p[k].edges[p[k+1].ID][1] #edge cost
    return c
