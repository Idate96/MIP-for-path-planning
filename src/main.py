from gurobipy import *
from vehicle import Vehicle
from obstacle import obstacle
import matplotlib.pyplot as plt
import random as rand
import numpy as np

if __name__ == '__main__':
    # Inputs to the generation of the obstacles
    num_obs = 3          # number of obstacles
    min_size = 1         # minimum size of the obstacle
    max_size = 2         # maximum size of the obstacle
    area_size = 50       # window size

    # Inputs to the generation of the vehicles
    vehicle_mass = 1     # mass of the vehicles
    v_max = 10           # maximum velocity of the vehicle
    f_max = 10           # maximum force experienced by a vehicle
    T = 30               # maximum time of travel
    dt = 0.1             # time step size
    steps = int(T/dt)    # number of steps
    wp = True            # switch for use of waypoints. True: waypoints can be used. False: function deactivated
    n_way_points = 1    # number of waypoints

    obs_coords = [[1,10,-0.1,40],     #array containing all obstacles in [x_min,x_max,y_min,y_max] format
                [20,30,10,51],
                [35,45,-1,45]
                ]   
    veh_coords = [[0,0,50,50],     #array containing all vehicles in [x_0,y_0,x_fin,y_fin] format
                [50,50,0,0]            
                ]   
    wp_coords = [[0,50],     #array containing all waypoint in [x_wp,y_wp] format
                [50,0]            
                ]  
                
    obstacles = []                                  #init array with obstacles
    for ob in obs_coords:                           #for every obstacle
        tmp = obstacle(ob[0], ob[1], ob[2], ob[3])  #local obstacle variable
        tmp.draw()                                  #draw local variable    
        obstacles.append(tmp)                       #attach obstacle to obstacle


    # Create initial and final positions

    num_vehicles = len(veh_coords)
    x0 = []; y0 = []                                 # initial positions for all vehicles
    x_fin = []; y_fin = []                           # final positions for all vehicles
    for i in range(num_vehicles):
        x0.append(veh_coords[i][0])
        y0.append(veh_coords[i][1])
        x_fin.append(veh_coords[i][2])
        y_fin.append(veh_coords[i][3])

    n_way_points = len(wp_coords)
    x_wp = []; y_wp = []                             # position of all waypoints of all vehicles
    if wp:                                           # if wp is True, waypoints are used
        for i in range(num_vehicles):
            x_dummy = []; y_dummy = []               # position of all waypoints of one vehicle
            for j in range(n_way_points):
                x_dummy.append(wp_coords[j][0])
                y_dummy.append(wp_coords[j][1])
                # x_dummy.append(area_size/2)
                # y_dummy.append(area_size/2)

            x_wp.append(x_dummy)
            y_wp.append(y_dummy)


    # Initialize model
    m = Model("ppl")

    # Create vehicles and add model main variables
    vehicles = []
    for i in range(num_vehicles):
        vehicles.append(Vehicle(vehicle_mass, dt, T, x0[i], y0[i], i, obstacles, m, v_max, f_max, area_size, x_fin[i], y_fin[i], wp, x_wp[i], y_wp[i]))

    # Add constraints and add model secondary variables
    for i in range(num_vehicles):
        vehicles[i].constrain(m, vehicles)

    # Obtaining the objective function
    total = 0                                # total number of time steps between all the vehicles (minimize)
    for veh in range(len(vehicles)):
        for i in range(steps):
            total += vehicles[veh].b[i] * i + vehicles[veh].fm[i]*0.0001


    m.setObjective(total, GRB.MINIMIZE)

    # Optimizing the model and obtaining the values of he parameters and the objective function
    m.optimize()
    m.getVars()

    # Plotting the results

    for i in range(num_vehicles):
        z = 0
        for k in range(steps):                 # obtaining time step at which vehicle reaches the final point
            Z = str(vehicles[i].b[k])
            if Z[-5] == "1":
                z = k
                break
        coords = np.zeros([z,2])
        for j in range(z):
            coords[j,:] = [vehicles[i].x[j].x,vehicles[i].y[j].x]
        for jj in range(len(x_wp[i])):                                                   #TODO:match plotting to style in paper
            plt.plot(x_wp[i][jj], y_wp[i][jj], 'x', markersize=15, color='g')            # plot the waypoints
        plt.plot(coords[:,0], coords[:,1], 'o')                                          # plot the trajectories of the vehicles
        plt.plot(vehicles[i].x[0].x, vehicles[i].y[0].x, 'x', markersize=15, color='r')  # plot the initial points
        plt.plot(vehicles[i].x_fin, vehicles[i].y_fin, 'x', markersize=15, color='b')    # plot the final points

    plt.show()
