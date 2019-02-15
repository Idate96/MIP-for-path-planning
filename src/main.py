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

# Inputs to the generation of the obstacles
obstacles = []
min_size = 1
max_size = 2
area_size = 20
v_max = 10
f_max = 10
num_obs = 1
i = 0

# Inputs to the generation of the vehicles
vehicles = []
num_vehicles = 2
vehicle_mass = 1
T = 30
dt = 0.1
steps = int(T/dt)
wp = True
n_way_points = 1

# Generate the obstacles

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

# Create initial and final positions
x0 = []; y0 = []
x_fin = []; y_fin = []
for i in range(num_vehicles):
    x0.append(rand.random() * area_size)
    y0.append(rand.random() * area_size)
    x_fin.append(rand.random() * area_size)
    y_fin.append(rand.random() * area_size)

x_wp = []; y_wp = []
if wp:
    for i in range(num_vehicles):
        x_dummy = []; y_dummy = []
        for j in range(n_way_points):
            x_dummy.append(rand.random() * area_size)
            y_dummy.append(rand.random() * area_size)
            x_dummy.append(area_size/2)
            y_dummy.append(20)

        x_wp.append(x_dummy)
        y_wp.append(y_dummy)


# Initialize model
m = Model("ppl")

# Create vehicles
i=0
while i<num_vehicles:
    vehicles.append(Vehicle(vehicle_mass,dt,T,x0[i],y0[i],i,obstacles,m,v_max,f_max,area_size, x_fin[i],y_fin[i], wp, x_wp[i], y_wp[i]))
    i+=1

# Add constraints
i = 0
while i<num_vehicles:
    vehicles[i].constrain(m, vehicles)
    i+=1

# Obtaining the objective function
total = 0
for veh in range(len(vehicles)):
    for i in range(steps):
        total += vehicles[veh].b[i] * i


m.setObjective(total, GRB.MINIMIZE)

# Optimizing the model and obtaining the values of he parameters and the obejective function
m.optimize()
m.getVars()
# Z = m.objVal

# Plotting the results
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
