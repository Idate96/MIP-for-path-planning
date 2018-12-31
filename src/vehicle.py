import numpy as np
from gurobipy import *

class Vehicle:
    
    def __init__(self, mass: float, dt: float, T: float, x0: float, y0:float, id: int):
        self.A = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.B = np.array([[0,0],[0,0],[dt/mass,0],[0,dt/mass]])
        self.steps = int(T/dt) #number of time steps
        self.id = id
        self.dt = dt
        self.x0 = x0
        self.y0 = y0
        self.m = mass

    def constrain(self,m,v_max,f_max,area_size, x_fin:float, y_fin:float):
        #add variable arrays, length=amount of steps
        self.x = m.addVars(self.steps,lb=0,ub=area_size)
        self.y = m.addVars(self.steps,lb=0,ub=area_size)
        self.vx = m.addVars(self.steps,lb=0,ub=v_max)
        self.vy = m.addVars(self.steps,lb=0,ub=v_max)
        self.fx = m.addVars(self.steps,lb=0,ub=f_max)
        self.fy = m.addVars(self.steps,lb=0,ub=f_max)

        #for every entry in the array, add constraint
        #x_n+1 = A*X_n + B*u_N
        m.addConstrs((self.x[i+1] == (self.x[i] +self.dt*self.vx[i]) for i in range(self.steps-1)), name='x_step_constr')  #x time step constraint
        m.addConstrs((self.y[i+1] == (self.y[i] +self.dt*self.vy[i]) for i in range(self.steps-1)), name='y_step_constr')  #y time step constraint
        m.addConstrs((self.vx[i+1] == (self.vx[i]+self.fx[i]*self.dt*self.m) for i in range(self.steps-1)), name='f_x_constr') #fx maximum acceleration constraint
        m.addConstrs((self.vy[i+1] == (self.vy[i]+self.fy[i]*self.dt*self.m) for i in range(self.steps-1)), name='f_y_constr') #fy maximum acceleration constraint

        #initial and final position
        m.addConstr(self.x[0] == self.x0, name='x_init_constr')
        m.addConstr(self.y[0] == self.y0, name='y_init_constr')
        m.addConstr(self.x[self.steps-1] == x_fin, name='x_fin_constr')
        m.addConstr(self.y[self.steps-1] == y_fin, name='y_fin_constr')

        #m.setObjective(sum(self.dt*self.vx+self.dt*self.vy),GRB.MINIMIZE) #TODO: minimize travel time instead of travel distance
        m.optimize()




    # def step(self, force_input: np.array):
    #     if (force_input.shape != (2,1)):
    #         raise Exception("force_input shape is not correct. Shoud be (2,1)")
        
    #     self.x = np.matmul(self.A,self.x) + np.matmul(self.B,force_input)
    #     self.time += self.dt
    #     self.history.append((self.x, self.time))

    