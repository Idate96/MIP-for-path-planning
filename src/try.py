from gurobipy import *

Nt = 100
dt = 0.1

m = Model('testmodel')

# vx = m.AddVar(vtype=GRB.NUMERIC, name='velocity_x')
# vy = m.AddVar(vtype=GRB.NUMERIC, name='velocity_y')
# ax = m.AddVar(vtype=GRB.NUMERIC, name='acc_x')
# ay = m.AddVar(vtype=GRB.NUMERIC, name='acc_y')
# x = m.AddVar(vtype=GRB.NUMERIC, name='x')
# y = m.AddVar(vtype=GRB.NUMERIC, name='y')


v = m.AddVars(2, Nt, vtype=GRB.CONTINUOUS, name='velocity')
a = m.AddVars(2, Nt, vtype=GRB.CONTINUOUS, name='acceleration')
p = m.AddVars(2, Nt, vtype=GRB.CONTINUOUS, name='position')



m.addConstrs((v[:, i+1] ==  v[:, i] + dt * a[:, i] for i in range(Nt-1)), name="dynamics_velocity")
m.addConstrs((p[:, i+1] ==  p[:, i] + dt * v[:, i] + 0.5 * dt * dt * a[:, i] for i in range(Nt-1)), name="dynamics_acceleration")
