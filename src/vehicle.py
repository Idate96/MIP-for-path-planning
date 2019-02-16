from gurobipy import *
import numpy as np

class Vehicle:
    M = 8
    def __init__(self, mass: float, dt: float, T: float, x0: float, y0: float, id: int, obstacles, m, v_max, f_max, area_size, x_fin: float, y_fin: float, wp, x_wp=None, y_wp=None):
        self.steps = int(T/dt)        # number of time steps
        self.id = id                  # vehicle id
        self.dt = dt                  # time step size
        self.x0 = x0                  # vehicle initial position x-coordinate
        self.y0 = y0                  # vehicle initial position y-coordinate
        self.x_fin = x_fin            # vehicle final position x-coordinate
        self.y_fin = y_fin            # vehicle final position y-coordinate
        self.m = mass                 # vehicle mass
        self.obstacles = obstacles    # environment list of obstacles
        self.v_max = v_max            # maximum velocity
        self.wp = wp                  # switch of waypoints
        self.x_wp = x_wp              # x-coordinate of waypoints
        self.y_wp = y_wp              # y-coordinate of waypoints
        self.f_max = f_max            # max force magnitude to be applied
        self.v_max = v_max            # max allowed velocity

        # Add variable arrays, length = amount of steps
        self.x = m.addVars(self.steps, lb=0, ub=area_size)     # x-ccordinate at each time step
        self.y = m.addVars(self.steps, lb=0, ub=area_size)     # y-coordinate at each time step
        self.vx = m.addVars(self.steps, lb=-v_max, ub=v_max)   # velocity x-component at each time step
        self.vy = m.addVars(self.steps, lb=-v_max, ub=v_max)   # velocity y-component at each time step
        self.fx = m.addVars(self.steps, lb=-f_max, ub=f_max)   # acceleration x-component at each time step
        self.fy = m.addVars(self.steps, lb=-f_max, ub=f_max)   # acceleration y-component at each time step
        self.fm = m.addVars(self.steps, lb=0,ub=f_max)

    def constrain(self, m, vehicles):
        # Add constrain to each variable in the respective array
        # Step position and velocity constraints
        m.addConstrs((self.x[i+1] == (self.x[i] + self.dt*self.vx[i]) for i in range(self.steps-1)))
        m.addConstrs((self.y[i+1] == (self.y[i] + self.dt*self.vy[i]) for i in range(self.steps-1)))
        m.addConstrs((self.vx[i+1] == (self.vx[i] + self.fx[i]*self.dt/self.m) for i in range(self.steps-1)))
        m.addConstrs((self.vy[i+1] == (self.vy[i] + self.fy[i]*self.dt/self.m) for i in range(self.steps-1)))

        # Initial velocity constraint
        m.addConstr(self.vx[0] == 0)
        m.addConstr(self.vy[0] == 0)

        for m_small in range(1,self.M+1):
            m.addConstrs((self.fx[i] * np.cos(2*np.pi * m_small / self.M) + self.fy[i] * np.sin(2*np.pi*m_small/self.M) <= self.fm[i] for i in range(self.steps)), name=("fm_cons_" + str(m_small)) )
        for m_small in range(1,self.M+1):
            m.addConstrs((self.vx[i] * np.cos(2*np.pi * m_small / self.M) + self.vy[i] * np.sin(2*np.pi*m_small/self.M) <= self.v_max for i in range(self.steps)), name=("vm_cons_" + str(m_small)) )
        for m_small in range(1,self.M+1):
            m.addConstrs((self.fx[i] * np.cos(2*np.pi * m_small / self.M) + self.fy[i] * np.sin(2*np.pi*m_small/self.M) <= self.f_max for i in range(self.steps)), name=("vm_cons_" + str(m_small)) )

        # Initial position constraint
        m.addConstr(self.x[0] == self.x0)
        m.addConstr(self.y[0] == self.y0)

        # Obstacle constraints
        R = 100000                  # high value factor
        d_obs = 1                   # minimum distance from obstacle required
        for obs in self.obstacles:

            c = m.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)

            m.addConstrs((self.x[i] - obs.x_max >= d_obs-R*c[0, i] for i in range(self.steps-1)))
            m.addConstrs((-self.x[i] + obs.x_min >= d_obs-R*c[1, i] for i in range(self.steps-1)))
            m.addConstrs((self.y[i] - obs.y_max >= d_obs-R*c[2, i] for i in range(self.steps-1)))
            m.addConstrs((-self.y[i] + obs.y_min >= d_obs-R*c[3, i] for i in range(self.steps-1)))

            m.addConstrs((c[0, i] + c[1, i] + c[2, i] + c[3, i] <= 3 for i in range(self.steps-1)))

        # Vehicle collision constraints
        d_veh = 1
        new_vehicles = vehicles[0:self.id]+vehicles[self.id+1:len(vehicles)]  # list of vehicles excluding current object
        for veh in new_vehicles:

            e = m.addVars(4, self.steps, lb=0, vtype=GRB.BINARY)

            m.addConstrs((self.x[i] - veh.x[i] >= d_veh - R * e[0, i] for i in range(self.steps - 1)))
            m.addConstrs((-self.x[i] + veh.x[i] >= d_veh - R * e[1, i] for i in range(self.steps - 1)))
            m.addConstrs((self.y[i] - veh.y[i] >= d_veh - R * e[2, i] for i in range(self.steps - 1)))
            m.addConstrs((-self.y[i] + veh.y[i] >= d_veh - R * e[3, i] for i in range(self.steps - 1)))

            m.addConstrs((e[0, i] + e[1, i] + e[2, i] + e[3, i] <= 3 for i in range(self.steps - 1)))

        # Final location constraint (used in the objective function)
        self.b = m.addVars(self.steps, lb=0, vtype=GRB.BINARY)
        for t_step in range(self.steps):

            m.addConstr(self.x[t_step] - self.x_fin <= R*(1 - self.b[t_step]))
            m.addConstr(self.x[t_step] - self.x_fin >= - R * (1 - self.b[t_step]))
            m.addConstr(self.y[t_step] - self.y_fin <= R * (1 - self.b[t_step]))
            m.addConstr(self.y[t_step] - self.y_fin >= - R * (1 - self.b[t_step]))

        m.addConstr(self.b.sum() == 1)

        # Waypoints constraints
        if self.wp:
            for i in range(len(self.x_wp)):

                self.k = m.addVars(self.steps, lb=0, vtype=GRB.BINARY)

                for j in range(self.steps):
                    # Constraint that vehicle must pass through waypoint before reaching destination
                    m.addConstr(self.k[j] <= 1 - quicksum(self.b[i] for i in range(j+1)))
                for t_step in range(self.steps):
                    m.addConstr(self.x[t_step] - self.x_wp[i] <= R * (1 - self.k[t_step]))
                    m.addConstr(self.x[t_step] - self.x_wp[i] >= - R * (1 - self.k[t_step]))
                    m.addConstr(self.y[t_step] - self.y_wp[i] <= R * (1 - self.k[t_step]))
                    m.addConstr(self.y[t_step] - self.y_wp[i] >= - R * (1 - self.k[t_step]))

                m.addConstr(self.k.sum() == 1)
