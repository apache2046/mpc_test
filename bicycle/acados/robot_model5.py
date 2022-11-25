from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, atan, tan


def export_robot_model() -> AcadosModel:
    model_name = "car_bicycle5_ode"
    
    #vehicle const
    lr = 1.4
    lf = 1.4

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    psi = SX.sym("psi")
    v = SX.sym("v")
    delta = SX.sym("delta")

    x = vertcat(x, y, psi, v, delta)

    # control
    a = SX.sym("a")
    delta_dot = SX.sym("delta_dot")
    u = vertcat(a, delta_dot)

    # parameters
    p = []

    # dynamics
    beta = atan(tan(delta) * lr / (lr + lf))
    f_expl = vertcat(
        v * cos(psi + atan(tan(delta) * lr / (lr + lf))),
        v * sin(psi + atan(tan(delta) * lr / (lr + lf))),
        v / lr * sin(atan(tan(delta) * lr / (lr + lf))),
        a - 0.02 * v,
        delta_dot,
    )

    # little inaccurate , convert some part as linear ops
    # f_expl = vertcat(
    #     v * cos(psi + delta) * lr / (lr + lf),
    #     v * sin(psi + delta) * lr / (lr + lf),
    #     v / lr * delta * lr / (lr + lf),
    #     a,
    #     delta_dot,
    # )


    model = AcadosModel()
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.p = p
    model.name = model_name

    return model
