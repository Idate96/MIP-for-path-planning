from gurobipy import *
from vehicle import Vehicle
from obstacle import obstacle
import numpy as np
import matplotlib.pyplot as plt

import utils

def entry():
    pass    

if __name__ == '__main__':
    entry()

obstacles = []
min_size = 1
max_size = 3
area_size = 100
num_obs = 100
i=0
while i<num_obs:
    intercept  = False
    x = obstacle(min_size,max_size,area_size)
    for o in obstacles:
        if(x.intersect(o)):
            intercept = True
            break
    
    if(intercept):
        continue
    
    obstacles.append(x)
    obstacles[i].draw()
    i+=1

plt.show()
print('Got here')