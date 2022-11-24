from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, atan, tan


def export_robot_model() -> AcadosModel:
    model_name = "car_bicycle3_ode"
    
    #vehicle const
    lr = 1.5
    lf = 1.5

    # set up states & controls
    x = SX.sym("x")
    y = SX.sym("y")
    psi = SX.sym("psi")
    v = SX.sym("v")
   

    x = vertcat(x, y, psi, v)

    # control
    a = SX.sym("a")
    delta = SX.sym("delta")
    u = vertcat(a, delta)

    # parameters
    p = []

    # dynamics
    # beta = atan(tan(delta) * lr / (lr + lf))
    f_expl = vertcat(
        v * cos(psi + atan(tan(delta) * lr / (lr + lf))),
        v * sin(psi + atan(tan(delta) * lr / (lr + lf))),
        v / lr * sin(atan(tan(delta) * lr / (lr + lf))),
        a,
    )

    model = AcadosModel()
    model.f_expl_expr = f_expl
    model.x = x
    model.u = u
    model.p = p
    model.name = model_name

    return model
