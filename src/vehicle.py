import numpy as np

class Vehicle(object):
    time = 0
    history = list()
    
    def __init__(self, mass: float, dt: float, x0: np.array, id: int):
        self.A = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.B = np.array([[0,0],[0,0],[dt/mass,0],[0,dt/mass]])
        self.x = x0
        self.id = id
        self.dt = dt

    def step(self, force_input: np.array):
        if (force_input.shape != (2,1)):
            raise Exception("force_input shape is not correct. Shoud be (2,1)")
        
        self.x = np.matmul(self.A,self.x) + np.matmul(self.B,force_input)
        self.time += self.dt
        self.history.append((self.x, self.time))

    