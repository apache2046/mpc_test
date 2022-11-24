import cvxpy as cp
import numpy as np


X0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Intitalize the states [x,y,v,th,th_d]
XN = np.array([3.0, 2.0, 0.0, 0.0, 0.0])
# XN = np.array([0.0, 1.0, 0.0, 0.0, 0.0])

N = 200  # Define the number of discretization steps
T = 2.0  # Define the prediction horizon
F_max = 10  # Define the max force allowed
T_max = 1.0 * 10
Theta_max = 1.5
x_min = 0
x_max = 100

# xdot = v * cos(theta), v * sin(theta), F, theta_d, T

Q_mat = 1 * np.diag([1e3, 1e3, 0, 1e2, 0.0])  # [x, y, x_d, th, th_d]
Qe_mat = 200 * np.diag([1e3, 1e3, 0, 1e3, 0.0])
R_mat = 0.01 * np.diag([1e-1, 1e-2])

n = 5
m = 2

x = cp.Variable((n, T + 1))
u = cp.Variable((m, T))

cost = 0
constr = []
for t in range(T):
    # cost += cp.sum_squares(x[:, t + 1]) + cp.sum_squares(u[:, t])
    cost += cp.quad_form(x[:, t+1], Q_mat) + cp.quad_form(u[:, t], R_mat)
    constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t], cp.norm(u[:, t], "inf") <= 1]

# sums problem objectives and concatenates constraints.
constr += [x[:, T] == XN, x[:, 0] == X0]
problem = cp.Problem(cp.Minimize(cost), constr)
problem.solve()