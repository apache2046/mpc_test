from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from robot_model2 import export_robot_model
import numpy as np
import scipy.linalg
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from common import pydraw

X0 = np.array([5, 4.0, 1.55, 0.0, 0.0, 0, 0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([3.0, 0.0, 0.0, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# XN = np.array([3.0, 2.0, 0.0, 0.0, 0.0])
XN = np.array([30.0, 10.0, 0.0, 0.0, 0.0, 0, 0])
# XN = np.array([0.0, 5.0, 0.0, 0.0, 0.0])
# XN = np.array([0.0, 0, 0.0, 0.0, 0.0])


N_horizon = 60#100  # Define the number of discretization steps
T_horizon = 2#4.0  # Define the prediction horizon
jerk_max = 2  # 
delta_jerk_max = 2
delta_max = 0.7
x_min =  0
x_max = 100

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost
    Q_mat = 0* np.diag([1, 1, 1, 0, 0.0, 0, 0])  # [x, y, psi, v, delta, a, delta_dot]
    Qe_mat = 1 * np.diag([1e3, 1e3, 1e0, 1e0, 0, 0, 0])
    R_mat = 0 * np.diag([1e-1, 1e-2])

    ocp.cost.cost_type = "LINEAR_LS"    #LINEAR_LS NONLINEAR_LS
    ocp.cost.cost_type_e = "LINEAR_LS"  #LINEAR_LS NONLINEAR_LS

    ny = nx + nu
    ny_e = nx

    ocp.cost.W_e = Qe_mat
    ocp.cost.W = scipy.linalg.block_diag(2 * Q_mat, R_mat)

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    # set constraints
    ocp.constraints.lbu = np.array([-jerk_max, -delta_jerk_max])
    ocp.constraints.ubu = np.array([+jerk_max, +delta_jerk_max])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lbx = np.array([x_min, -delta_max])
    ocp.constraints.ubx = np.array([x_max, +delta_max])
    ocp.constraints.idxbx = np.array([0, 4])

    ocp.constraints.x0 = X0

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


def closed_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    ##########################
    # ocp.solver_options.qp_solver_cond_N = 1
    #########################
    print("G##", ocp.model.name)
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

    # prepare simulation
    Nsim = 400
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

    # closed loop
    for i in range(Nsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref
        for j in range(N_horizon):
            yref = np.concatenate([XN, [0,0]])
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = XN
        acados_ocp_solver.set(N_horizon, "yref", yref_N)

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
            raise Exception(
                f"acados1 acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )

        if status == 2:
            print(
                f"acados2 acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )
        simU[i, :] = acados_ocp_solver.get(0, "u")

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

    # plot results
    # plot_robot(
    #     np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [F_max, None], simU, simX
    # )

    #print(simX)
    pydraw(simX, simU, X0, XN)

if __name__ == "__main__":
    closed_loop_simulation()

