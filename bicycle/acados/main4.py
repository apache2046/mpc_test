from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from robot_model4 import export_robot_model
import numpy as np
import scipy.linalg
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from common4 import pydraw
from casadi import SX, vertcat, sin, cos, atan, tan, log, sqrt

X0 = np.array([5, 4.0, 1.55, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([10.0, 0.0, 1.0, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([10.0, 0.0, 0.5, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([20.0, 0.0, 1.5, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([0.0, 2.5, 0, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([5, 4.0, 2.55, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([5, 18.0, 4.55, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([5, 11.0, 1.6, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]

# X0 = np.array([10.0, 20, 0.0, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]

# XN = np.array([3.0, 2.0, 0.0, 0.0, 0.0])
# XN = np.array([30.0, 10.0, 0.0, 0.0, 0.0])
XN = np.array([0, 3.0, 0, 0.0, 0.0])
# XN = np.array([0.0, 0, 0.0, 0.0, 0.0])


N_horizon = 240#100  # Define the number of discretization steps
T_horizon = 8#4.0  # Define the prediction horizon
a_max = 4  # 
delta_dot_max = 2
delta_max = 0.6
x_min =  -0.5
x_max = 100
v_max = 8

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.size()[0] + 1
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = 1* np.diag([1, 1, 1, 0, 0.0, 1])  # [x, y, psi, v, delta, dis0]
    Qe_mat = 1 * np.diag([1e3, 1e3, 1e3, 1e0, 0, 1])
    R_mat = 0 * np.diag([1e-1, 1e-2])

    ocp.cost.cost_type = "NONLINEAR_LS"    #LINEAR_LS NONLINEAR_LS
    ocp.cost.cost_type_e = "NONLINEAR_LS"  #LINEAR_LS NONLINEAR_LS

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Qe_mat
    ocp.cost.W = scipy.linalg.block_diag(2 * Q_mat, R_mat)

    # ocp.cost.Vx = np.zeros((ny, nx))
    # ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    # Vu = np.zeros((ny, nu))
    # Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    # ocp.cost.Vu = Vu

    # ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([-a_max, -delta_dot_max])
    ocp.constraints.ubu = np.array([+a_max, +delta_dot_max])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lbx = np.array([x_min, -v_max, -delta_max])
    ocp.constraints.ubx = np.array([x_max, v_max, +delta_max])
    ocp.constraints.idxbx = np.array([0, 3, 4])

    ocp.constraints.x0 = X0


    #####
    obs0_x, obs0_y, obs0_r, obs1_x, obs1_y, obs1_r, obs2_x, obs2_y, obs2_r, obs3_x, obs3_y, obs3_r = ocp.model.p[0], \
        ocp.model.p[1], ocp.model.p[2], ocp.model.p[3], ocp.model.p[4], ocp.model.p[5], ocp.model.p[6], ocp.model.p[7], \
        ocp.model.p[8], ocp.model.p[9], ocp.model.p[10], ocp.model.p[10]
    
    ego_x, ego_y, ego_psi, ego_v, ego_delta = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2], ocp.model.x[3], ocp.model.x[4]
    u0, u1 = ocp.model.u[0], ocp.model.u[1]
    costs = [ego_x, ego_y, ego_psi, ego_v, ego_delta, \
            # -log((ego_x-obs0_x)**2 + (ego_y-obs0_y)**2 - obs0_r ** 2), \
            # -log((ego_x-obs0_x)**2 + (ego_y-obs0_y)**2),
            1 / ((ego_x-obs0_x)**2 + (ego_y-obs0_y)**2),
            # 0,
            u0, u1
            ]
    ocp.model.cost_y_expr = vertcat(*costs)
    ocp.model.cost_y_expr_e = vertcat(*costs[:-2])
    ocp.parameter_values = np.array([-100, -100, 0, -100, -100, 0, -100, -100, 0, -100, -100, 0])
    
    #Q = np.diag(np.zeros(1))
    #QR = np.diag(np.zeros(1))
    #ocp.cost.W = QR
    #ocp.cost.W_e = Q

    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES PARTIAL_CONDENSING_HPIPM
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "ERK" #"IRK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # SQP_RTI, SQP
    ocp.solver_options.nlp_solver_max_iter = 800 * 4
    # ocp.solver_options.levenberg_marquardt = 1e-2

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp


def reset_mpc(acados_ocp_solver):
    acados_ocp_solver.load_iterate(filename='aaa')

def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    ##########################
    # ocp.solver_options.qp_solver_cond_N = 1
    #########################
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

    # prepare simulation
    Nsim = 600
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    simX = np.ndarray((Nsim + 1, nx))
    simU = np.ndarray((Nsim, nu))

    xcurrent = X0
    simX[0, :] = xcurrent

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(xcurrent.shape))
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    acados_ocp_solver.store_iterate(filename='aaa', overwrite=True)

    # closed loop
    total_steps = 0
    for i in range(Nsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref
        for j in range(N_horizon):
            yref = np.concatenate([XN, [0,0,0]])
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = np.concatenate([XN, [0]])
        acados_ocp_solver.set(N_horizon, "yref", yref_N)

        #####
        params = np.array([2, 2, 1, -100, -100, 0, -100, -100, 0, -100, -100, 0])
        for j in range(N_horizon + 1):
           acados_ocp_solver.set(i, 'p', params)

        # solve ocp
        status = acados_ocp_solver.solve()

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            # plot_robot(
            #     np.linspace(0, T_horizon / N_horizon * i, i + 1),
            #     F_max,
            #     simU[:i, :],
            #     simX[: i + 1, :],
            # )
            # raise Exception(
            #     f"acados1 acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            # )
            print(
                f"acados1 acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )
            simU[i, :] = [0, 0]
            reset_mpc(acados_ocp_solver)
        else:
            simU[i, :] = acados_ocp_solver.get(0, "u")

        if status == 2:
            print(
                f"acados2 acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )


        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", simU[i, :])

        status = acados_integrator.solve()
        if status != 0:
            raise Exception(
                f"acados3 integrator returned status {status} in closed loop instance {i}"
            )

        # update state
        xcurrent = acados_integrator.get("x")
        simX[i + 1, :] = xcurrent

        total_steps += 1

        if np.linalg.norm(xcurrent - XN) < 0.1:
            break

    # plot results
    # plot_robot(
    #     np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [F_max, None], simU, simX
    # )

    print(simX, simU)
    pydraw(simX, simU, X0, XN, total_steps)

if __name__ == "__main__":
    closed_loop_simulation()

