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

##create obstacle array
obstacles = []
min_size = 5
max_size = 10
area_size = 1000
v_max = 10
f_max = 10
num_obs = 5
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

##create vehicles
vehicles = []
num_vehicles = 1
vehicle_mass = 50
i=0
T = 30
dt = 0.1
while i<num_vehicles:
    vehicles.append(Vehicle(vehicle_mass,dt,T,area_size,area_size,i))
    i+=1

#initialize model
m = Model("ppl")

#add constraints
i = 0
while i<num_vehicles:
    vehicles[i].constrain(m,v_max,f_max,area_size,0,0)
    i+=1
    
#optimize

#plot
i=0
# while i<num_vehicles:
#     plt.plot(vehicles[i].x[0],vehicles[i].y[0],'o')
#     i+=1
plt.show()
