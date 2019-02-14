import numpy as np
from gurobipy import *

class Vehicle:
    
    def __init__(self, mass: float, dt: float, T: float, x0: float, y0:float, id: int,obstacles, m, v_max, f_max, area_size, x_fin:float, y_fin:float,):
        self.A = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.B = np.array([[0,0],[0,0],[dt/mass,0],[0,dt/mass]])
        self.steps = int(T/dt) # number of time steps
        self.id = id
        self.dt = dt
        self.x0 = x0
        self.y0 = y0
        self.m = mass
        self.obstacles = obstacles
        self.v_max = v_max
        self.x_fin = x_fin
        self.y_fin = y_fin

        # add variable arrays, length=amount of steps
        self.x = m.addVars(self.steps, lb=0, ub=area_size)
        self.y = m.addVars(self.steps, lb=0, ub=area_size)
        self.vx = m.addVars(self.steps, lb=-v_max, ub=v_max)
        self.vy = m.addVars(self.steps, lb=-v_max, ub=v_max)
        self.fx = m.addVars(self.steps, lb=-f_max, ub=f_max)
        self.fy = m.addVars(self.steps, lb=-f_max, ub=f_max)

    def constrain(self, m, vehicles):


        #for every entry in the array, add constraint
        m.addConstrs((self.x[i+1] == (self.x[i] + self.dt*self.vx[i]) for i in range(self.steps-1)))  #x time step constraint
        m.addConstrs((self.y[i+1] == (self.y[i] + self.dt*self.vy[i]) for i in range(self.steps-1)))  #y time step constraint
        m.addConstrs((self.vx[i+1] == (self.vx[i] + self.fx[i]*self.dt/self.m) for i in range(self.steps-1))) #fx maximum acceleration constraint
        m.addConstrs((self.vy[i+1] == (self.vy[i] + self.fy[i]*self.dt/self.m) for i in range(self.steps-1))) #fy maximum acceleration constraint

        m.addConstr(self.vx[0] == 0)
        m.addConstr(self.vy[0] == 0)


        m.addConstrs(((self.vx[i]*self.vx[i]+self.vy[i]*self.vy[i]) <= self.v_max*self.v_max for i in range(self.steps-1)))

        
        #initial and final position
        m.addConstr(self.x[0] == self.x0)
        m.addConstr(self.y[0] == self.y0)

        # add constraints for obstacles
        R = 100000
        d = 1
        for obs in self.obstacles:

            c = m.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)

            xmin = obs.x-obs.size
            xmax = obs.x+obs.size
            ymin = obs.y-obs.size
            ymax = obs.y+obs.size

            m.addConstrs((self.x[i] - xmax >= d-R*c[0, i] for i in range(self.steps-1)))
            m.addConstrs((-self.x[i] + xmin >= d-R*c[1, i] for i in range(self.steps-1)))
            m.addConstrs((self.y[i] - ymax >= d-R*c[2, i] for i in range(self.steps-1)))
            m.addConstrs((-self.y[i] + ymin >= d-R*c[3, i] for i in range(self.steps-1)))

            m.addConstrs((c[0,i] + c[1,i] + c[2,i] + c[3,i] <= 3 for i in range(self.steps-1)))

        # new_vehicles = vehicles[0:self.id]+vehicles[self.id+1:len(vehicles)]
        # print(new_vehicles)
        # for veh in range(len(new_vehicles)):
        #
        #     e = m.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)
        #
        #     m.addConstrs((self.x[i] - vehicles[veh].x[i] >= d - R * e[0, i] for i in range(self.steps - 1)))
        #     m.addConstrs((-self.x[i] - vehicles[veh].x[i] >= d - R * e[1, i] for i in range(self.steps - 1)))
        #     m.addConstrs((self.y[i] - vehicles[veh].y[i] >= d - R * e[2, i] for i in range(self.steps - 1)))
        #     m.addConstrs((-self.y[i] - vehicles[veh].y[i] >= d - R * e[3, i] for i in range(self.steps - 1)))
        #
        #     m.addConstrs((e[0, i] + e[1, i] + e[2, i] + e[3, i] <= 3 for i in range(self.steps - 1)))

        self.b = m.addVars(self.steps, lb=0, vtype=GRB.BINARY)
        for t_step in range(self.steps):

            m.addConstr(self.x[t_step] - self.x_fin <= R*(1 - self.b[t_step]))
            m.addConstr(self.x[t_step] - self.x_fin >= - R * (1 - self.b[t_step]))
            m.addConstr(self.y[t_step] - self.y_fin <= R * (1 - self.b[t_step]))
            m.addConstr(self.y[t_step] - self.y_fin >= - R * (1 - self.b[t_step]))

        m.addConstr(self.b.sum() == 1)

        m.setObjective(quicksum(i*self.b[i] for i in range(self.steps)),  GRB.MINIMIZE)

        m.optimize()
        m.getVars()
        # self.Z = m.objVal


        # print("OBJJJJ:", self.Z)
