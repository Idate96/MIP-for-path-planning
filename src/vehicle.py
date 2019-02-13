import numpy as np
from gurobipy import *

class Vehicle:
    
    def __init__(self, mass: float, dt: float, T: float, x0: float, y0:float, id: int,obstacles):
        self.A = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.B = np.array([[0,0],[0,0],[dt/mass,0],[0,dt/mass]])
        self.steps = int(T/dt) #number of time steps
        self.id = id
        self.dt = dt
        self.x0 = 0
        self.y0 = 0
        self.m = mass
        self.obstacles = obstacles

    def constrain(self,m,v_max,f_max,area_size, x_fin:float, y_fin:float):
        #add variable arrays, length=amount of steps
        self.x = m.addVars(self.steps,lb=0,ub=area_size, name='x')
        self.y = m.addVars(self.steps,lb=0,ub=area_size, name='y')
        self.vx = m.addVars(self.steps,lb=0,ub=v_max, name='vx')
        self.vy = m.addVars(self.steps,lb=0,ub=v_max, name='vy')
        self.fx = m.addVars(self.steps,lb=0,ub=f_max, name='fx')
        self.fy = m.addVars(self.steps,lb=0,ub=f_max, name='fy')

        #for every entry in the array, add constraint
        #x_n+1 = A*X_n + B*u_N
        m.addConstrs((self.x[i+1] == (self.x[i] +self.dt*self.vx[i]) for i in range(self.steps-1)), name='x_step_constr')  #x time step constraint
        m.addConstrs((self.y[i+1] == (self.y[i] +self.dt*self.vy[i]) for i in range(self.steps-1)), name='y_step_constr')  #y time step constraint
        m.addConstrs((self.vx[i+1] == (self.vx[i]+self.fx[i]*self.dt*self.m) for i in range(self.steps-1)), name='f_x_constr') #fx maximum acceleration constraint
        m.addConstrs((self.vy[i+1] == (self.vy[i]+self.fy[i]*self.dt*self.m) for i in range(self.steps-1)), name='f_y_constr') #fy maximum acceleration constraint
        m.addConstrs((self.fx[i] <= f_max for i in range(self.steps-1)), name='f_x_max')
        m.addConstrs((self.fy[i] <= f_max for i in range(self.steps-1)), name='f_y_max')
        m.addConstrs((self.fx[i] >= -f_max for i in range(self.steps-1)), name='f_x_max')
        m.addConstrs((self.fy[i] >= -f_max for i in range(self.steps-1)), name='f_y_max')
        m.addConstr(self.vx[0] == 0, name='vx_init_constr')
        m.addConstr(self.vy[0] == 0, name='vy_init_constr')
        # m.addConstr(self.vx[self.steps-1] == 0, name='vx_init_final')
        # m.addConstr(self.vy[self.steps-1] == 0, name='vy_init_final')

        m.addConstrs(((self.vx[i]*self.vx[i]+self.vy[i]*self.vy[i]) <= v_max*v_max for i in range(self.steps-1)), name='f_x_max')

        
        #initial and final position
        m.addConstr(self.x[0] == self.x0, name='x_init_constr')
        m.addConstr(self.y[0] == self.y0, name='y_init_constr')
        m.addConstr(self.x[self.steps-1] == x_fin, name='x_fin_constr')
        m.addConstr(self.y[self.steps-1] == y_fin, name='y_fin_constr')

        #add constraints for obstacles
        for obs in self.obstacles:

            c = m.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)

            xmin = obs.x-obs.size
            xmax = obs.x+obs.size
            ymin = obs.y-obs.size
            ymax = obs.y+obs.size

            m.addConstrs((self.x[i] - xmax >= -100000*c[0,i] for i in range(self.steps-1)))
            m.addConstrs((-self.x[i] + xmin >= -100000*c[1,i] for i in range(self.steps-1)))
            m.addConstrs((self.y[i] - ymax >= -100000*c[2,i] for i in range(self.steps-1)))
            m.addConstrs((-self.y[i] + ymin >= -100000*c[3,i] for i in range(self.steps-1)))

            m.addConstrs((c[1,i] + c[0,i] + c[2,i] + c[3,i] <= 3 for i in range(self.steps-1)))

        obj = 0
        for i in range(self.steps-1):
            if(self.x[i] == x_fin and self.y[i] == y_fin):
                obj += 0
            else:
                obj += self.dt
        #set vel
        m.setObjective(obj,GRB.MINIMIZE) #TODO: minimize travel time instead of travel distance
        m.optimize()
        m.getVars()

    # def step(self, force_input: np.array):
    #     if (force_input.shape != (2,1)):
    #         raise Exception("force_input shape is not correct. Shoud be (2,1)")
        
    #     self.x = np.matmul(self.A,self.x) + np.matmul(self.B,force_input)
    #     self.time += self.dt
    #     self.history.append((self.x, self.time))

    