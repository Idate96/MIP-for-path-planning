from gurobipy import *
from vehicle import Vehicle
from obstacle import obstacle
import numpy as np
import matplotlib.pyplot as plt
import random as rand


import utils

def entry():
    pass    

if __name__ == '__main__':
    entry()

##create obstacle array
obstacles = []
min_size = 1
max_size = 3
area_size = 20
v_max = 10
f_max = 10
num_obs = 10
i = 0
while i<num_obs:
    intercept = False
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
num_vehicles = 2
vehicle_mass = 1
i = 0
T = 30
dt = 0.1
steps = int(T/dt)

x0 = []; y0 = []
x_fin = []; y_fin = []
for i in range(num_vehicles):
    x0.append(rand.random() * area_size)
    y0.append(rand.random() * area_size)
    x_fin.append(rand.random() * area_size)
    y_fin.append(rand.random() * area_size)

#initialize model
m = Model("ppl")
i=0
while i<num_vehicles:
    vehicles.append(Vehicle(vehicle_mass,dt,T,x0[i],y0[i],i,obstacles,m,v_max,f_max,area_size, x_fin[i],y_fin[i]))
    i+=1

#add constraints
i = 0
while i<num_vehicles:
    vehicles[i].constrain(m, vehicles)
    i+=1




# m.setObjective(quicksum(i*vehicles[0].b[i]for i in range(steps)),  GRB.MINIMIZE)
#
# m.optimize()
# m.getVars()
# Z = m.objVal

# plot
i = 0
while i<num_vehicles:
    for k in range(steps):
        Z = str(vehicles[i].b[k])
        if Z[-5] == "1":
            z = k
            break
    for j in range(z):
        plt.plot(vehicles[i].x[j].x, vehicles[i].y[j].x, 'o')
    plt.plot(vehicles[i].x[0].x, vehicles[i].y[0].x, 'x', markersize = 15, color='r')
    plt.plot(vehicles[i].x_fin, vehicles[i].y_fin, 'x', markersize = 15, color='b')

    i+=1
plt.show()
