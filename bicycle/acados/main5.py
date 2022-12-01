from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from robot_model5 import export_robot_model
import numpy as np
import scipy.linalg
import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from common5 import pydraw

N_horizon = 60#100  # Define the number of discretization steps
T_horizon = N_horizon / 30#4.0  # Define the prediction horizon
N_maxsim = 600
a_max = 3  # 
delta_dot_max = 2
delta_max = 0.6
x_min =  -0.2
x_max = 100
v_max = 8

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
    Q_mat = 0.01 * np.diag([1, 1, 1, 0, 0.0])  # [x, y, psi, v, delta]
    Qe_mat = 1 * np.diag([1e3, 1e3, 1e3, 1e0, 0])
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
    ocp.constraints.lbu = np.array([-a_max, -delta_dot_max])
    ocp.constraints.ubu = np.array([+a_max, +delta_dot_max])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lbx = np.array([x_min, -v_max, -delta_max])
    ocp.constraints.ubx = np.array([x_max, v_max, +delta_max])
    ocp.constraints.idxbx = np.array([0, 3, 4])

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


def reset_mpc(acados_ocp_solver):
    acados_ocp_solver.load_iterate(filename='aaa')


acados_ocp_solver:AcadosOcpSolver = None
acados_integrator:AcadosSimSolver = None

def init_solver():
    global acados_ocp_solver
    global acados_integrator

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
    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", np.zeros((nx,)))
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    acados_ocp_solver.store_iterate(filename='aaa', overwrite=True)

def solve_sim(xStart, xEnd):
    # closed loop
    global acados_ocp_solver
    total_steps = 0
    # print(acados_ocp_solver.get(0, "x"), type(acados_ocp_solver.get(0, "x")), acados_ocp_solver.get(0, "x").sh)
    nx = acados_ocp_solver.get(0, "x").size
    nu = acados_ocp_solver.get(0, "u").size

    simX = np.ndarray((N_maxsim + 1, nx))
    simU = np.ndarray((N_maxsim, nu))

    xcurrent = xStart
    for i in range(N_maxsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # update yref
        for j in range(N_horizon):
            yref = np.concatenate([xEnd, [0, 0]])
            acados_ocp_solver.set(j, "yref", yref)
        yref_N = xEnd
        acados_ocp_solver.set(N_horizon, "yref", yref_N)

        # solve ocp
        status = acados_ocp_solver.solve()

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
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

        error = xcurrent - XN
        error[2] *= 10
        error[3:] = 0 
        if np.linalg.norm(error) < 0.5:
            break
    return simX[1:total_steps+1], simU[:total_steps]


X0 = np.array([33.0, 13.0, 0, 0.0, 0.0])  # Intitalize the states [x, y, psi, v, delta]
# X0 = np.array([19.2, 1.5, np.pi / 2, 0.0, 0.0])
XNs = [ np.array([18.2, 12.0, 1, 0.0, 0.0]),
        np.array([16.8, 1.5, np.pi / 2, 0.0, 0.0]),
        np.array([17.4, 5, np.pi / 2, 0.0, 0.0]),
        np.array([18.0, 1.5, np.pi / 2, 0.0, 0.0]),
        np.array([18.6, 5, np.pi / 2, 0.0, 0.0]),
# XNs = [
        np.array([19.2, 1.5, np.pi / 2, 0.0, 0.0]),
        np.array([17.5, 5.3, 1.8, 0.0, 0.0]),
        np.array([16.8, 9.5, np.pi / 2, 0.0, 0.0]),
        np.array([4, 13, np.pi, 0.0, 0.0]),
        np.array([17.7, 12, 2.14, 0.0, 0.0]),
        np.array([19.2, 1.5, np.pi / 2, 0.0, 0.0]),
      ]

if __name__ == "__main__":
    # closed_loop_simulation()
    init_solver()
    simx = [X0]
    simu = []
    arrow_idx = []
    for idx, XN in enumerate(XNs):
        x, u = solve_sim(simx[-1], XN)
        simx.extend(x)
        simu.extend(u)
        tmp = [idx] * len(x)
        arrow_idx.extend(tmp)

    simx = np.array(simx)
    simu = np.array(simu)
    
    pydraw(simx, simu, [X0, *XNs], arrow_idx)



