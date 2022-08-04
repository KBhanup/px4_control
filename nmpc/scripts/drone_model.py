from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos


def drone_model():
    """
      Defines the parameters and dynamics of the drone model
    """

    model_name = 'drone_w_disturbances'

    # State
    px = SX.sym('px')    # Position x
    py = SX.sym('py')    # Position y
    pz = SX.sym('pz')    # Position z
    vx = SX.sym('vx')    # Velocity x
    vy = SX.sym('vy')    # Velocity y
    vz = SX.sym('vz')    # Velocity z
    qx = SX.sym('qx')    # Roll
    qy = SX.sym('qy')    # Pitch
    qz = SX.sym('qz')    # Yaw
    x = vertcat(px, py, pz,
                vx, vy, vz,
                qx, qy, qz)

    # Input
    u1 = SX.sym('u1')  # Yaw rate
    u2 = SX.sym('u2')  # Pitch
    u3 = SX.sym('u3')  # Roll
    u4 = SX.sym('u4')  # Thrust
    u = vertcat(u1, u2, u3, u4)

    # Model parameters
    tp = SX.sym('tp')    # Roll time constant
    kp = SX.sym('kp')    # Roll gain
    tr = SX.sym('tr')    # Pitch time constant
    kr = SX.sym('kr')    # Pitch gain
    dx = SX.sym('dx')    # Damping x
    dy = SX.sym('dy')    # Damping y
    dz = SX.sym('dz')    # Damping z
    fdx = SX.sym('fdx')  # Disturbance force x
    fdy = SX.sym('fdy')  # Disturbance force y
    fdz = SX.sym('fdz')  # Disturbance force z
    kth = SX.sym('kth')  # Thrust coefficients
    g = SX.sym('g')      # Gravity
    p = vertcat(tp, kp,
                tr, kr,
                dx, dy, dz,
                fdx, fdy, fdz,
                kth, g)

    # F_impl
    px_dot = SX.sym('px_dot')
    py_dot = SX.sym('py_dot')
    pz_dot = SX.sym('pz_dot')
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')
    qx_dot = SX.sym('qx_dot')
    qy_dot = SX.sym('qy_dot')
    qz_dot = SX.sym('qz_dot')
    x_dot = vertcat(px_dot, py_dot, pz_dot,
                    vx_dot, vy_dot, vz_dot,
                    qx_dot, qy_dot, qz_dot)

    # Dynamics
    dpx = vx
    dpy = vy
    dpz = vz
    dvx = dx * vx + (cos(qz)*sin(qy)*cos(qx) + sin(qz)*sin(qx)) * kth * u4 + fdx
    dvy = dy * vy + (sin(qz)*sin(qy)*cos(qx) - cos(qz)*sin(qx)) * kth * u4 + fdy
    dvz = dz * vz + (cos(qy)*cos(qx)) * kth * u4 + g + fdz
    dqx = (kr * u3 - qx) / tr
    dqy = (kp * u2 - qy) / tp
    dqz = u1

    f_expl = vertcat(dpx, dpy, dpz,
                     dvx, dvy, dvz,
                     dqx, dqy, dqz)
    f_impl = x_dot - f_expl

    # Acados model
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = x_dot
    model.u = u
    model.p = p
    model.name = model_name

    return model
