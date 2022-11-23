from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from robot_model import export_robot_model
import numpy as np
import scipy.linalg
from utils import plot_robot


X0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Intitalize the states [x,y,v,th,th_d]
XN = np.array([3.0, 2.0, 0.0, 0.0, 0.0])

N_horizon = 200  # Define the number of discretization steps
T_horizon = 2.0  # Define the prediction horizon
F_max = 10 * 0.2 # Define the max force allowed
F_min = -10
T_max = 1.0 * 1
T_min = -1.0



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
    Q_mat = 1 * np.diag([1e3, 1e3, 1e3, 1e3, 0.0])  # [x, y, x_d, th, th_d]
    Qe_mat = 200 * np.diag([1e3, 1e3, 1e3, 1e3, 0.0])
    R_mat = 0.1 * np.diag([1e-1, 1e-2])

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

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
    ocp.constraints.lbu = np.array([-F_max, -T_max])
    ocp.constraints.ubu = np.array([+F_max, +T_max])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lbx = np.array([-1.5])
    ocp.constraints.ubx = np.array([1.5])
    ocp.constraints.idxbx = np.array([2])

    ocp.constraints.x0 = X0

    # set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = "IRK"
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
    acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )
    acados_integrator = AcadosSimSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

    # prepare simulation
    Nsim = 300
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
            plot_robot(
                np.linspace(0, T_horizon / N_horizon * i, i + 1),
                F_max,
                simU[:i, :],
                simX[: i + 1, :],
            )
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
    plot_robot(
        np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [F_max, None], simU, simX
    )

    #print(simX)
    pydraw(simX, simU)
     
def pydraw(X, U):

    import pyglet
    from pyglet import shapes

    win_width, win_height = 640, 480
    window = pyglet.window.Window(win_width, win_height)
    batch = pyglet.graphics.Batch()

    car_image = pyglet.image.load('car.png')
    car_image.anchor_x = 32
    car_image.anchor_y = 32
    car = pyglet.sprite.Sprite(car_image, x=50, y=50)
    car.rotation = 90

    bg_color = shapes.Rectangle(0, 0, 960, 540, color=(255, 255, 255), batch=batch)
    bg_lines = []
    for i in list(range(win_width//32)):
        bg_lines.append(shapes.Line((i+1)*32, 0, (i+1)*32, win_height, width=1, color=(180, 180, 180), batch=batch))
    for i in list(range(win_height//32)):
        bg_lines.append(shapes.Line(0, (i+1)*32, win_width, (i+1)*32, width=1, color=(180, 180, 180), batch=batch))
    

    start_point = shapes.Circle(100 + X0[0] * 100, 100 + X0[1] * 100, 3, color=(200, 40, 40), batch=batch)
    end_point   = shapes.Circle(100 + XN[0] * 100, 100 + XN[1] * 100, 3, color=(200, 40, 40), batch=batch)

    labelX = pyglet.text.Label('EgoX:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=100, batch=batch)
    labelY = pyglet.text.Label('EgoY:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=80, batch=batch)
    labelTheta = pyglet.text.Label('Theta:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=60, batch=batch)
    labelF = pyglet.text.Label('F:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=400, y=100, batch=batch)
    labelT = pyglet.text.Label('T:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=400, y=80, batch=batch)
    idx = 0
    frame = 0
    traj = []
    @window.event
    def on_draw():
        nonlocal idx
        nonlocal frame
        nonlocal traj
        frame += 1
        if frame % 4 == 0:
            idx = (idx + 1) % len(U)
            if idx == 0:
                traj = []
        window.clear()
        labelX.text = f"EgoX:  {X[idx, 0]:.2f}"
        labelY.text = f"EgoY:  {X[idx, 1]:.2f}"
        labelTheta.text = f"Theta:  {X[idx, 3]:.2f}"
        labelF.text = f"F:  {U[idx, 0]:.2f}"
        labelT.text = f"T:  {U[idx, 1]:.2f}"
        batch.draw()

        if len(traj) > 1:
            batch1 = pyglet.graphics.Batch()
            tmp = []
            for x,y in traj:
                tmp.append(shapes.Circle(x, y, 1, color=(160, 40, 40), batch=batch1))
            batch1.draw()
                


        car.x = 100 + X[idx ,0] * 100
        car.y = 100 + X[idx ,1] * 100
        car.rotation = 90 - X[idx ,3] * 180 / np.pi
        car.draw()
        if len(traj) == 0 or traj[-1] != (car.x, car.y):
            traj.append((car.x, car.y))
        



    pyglet.app.run()

if __name__ == "__main__":
    closed_loop_simulation()

