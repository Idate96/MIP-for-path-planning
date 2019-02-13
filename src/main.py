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
min_size = 1
max_size = 3
area_size = 50
v_max = 10
f_max = 1
num_obs = 15
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
steps = int(T/dt)
while i<num_vehicles:
    vehicles.append(Vehicle(vehicle_mass,dt,T,area_size,area_size,i,obstacles))
    i+=1

#initialize model
m = Model("ppl")

#add constraints
i = 0
while i<num_vehicles:
    vehicles[i].constrain(m,v_max,f_max,area_size,area_size,area_size)
    i+=1
    
#optimize

#plot
i=0
while i<num_vehicles:
    for j in range(int(T/dt)):
        plt.plot(vehicles[i].x[j].x,vehicles[i].y[j].x,'o')
    i+=1
plt.show()
