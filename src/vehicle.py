import numpy as np
from gurobipy import *

class Vehicle:
    
    def __init__(self, mass: float, dt: float, T: float, x0: float, y0:float, id: int):
        self.A = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.B = np.array([[0,0],[0,0],[dt/mass,0],[0,dt/mass]])
        self.steps = int(T/dt) #number of time steps
        self.x = x0*np.ones(self.steps)
        self.y = y0*np.ones(self.steps)
        self.vx = np.zeros(self.steps)
        self.vy = np.zeros(self.steps)
        self.id = id
        self.dt = dt

    def constrain(self,m):
        m.addVars(self.x,name='x')
        # y_opt = m.addVars(self.y)
        # vx_opt = m.addVars(self.vx)
        # vy_opt = m.addVars(self.vy)


    # def step(self, force_input: np.array):
    #     if (force_input.shape != (2,1)):
    #         raise Exception("force_input shape is not correct. Shoud be (2,1)")
        
    #     self.x = np.matmul(self.A,self.x) + np.matmul(self.B,force_input)
    #     self.time += self.dt
    #     self.history.append((self.x, self.time))

    