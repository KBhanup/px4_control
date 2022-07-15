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
    qr = SX.sym('qr')    # Roll
    qp = SX.sym('qp')    # Pitch
    qy = SX.sym('qy')    # Yaw
    qyr = SX.sym('qyr')  # Yaw rate

    x = vertcat(px, py, pz,
                vx, vy, vz,
                qr, qp, qy,
                qyr)

    # Input
    u1 = SX.sym('u1')  # Yaw rate command
    u2 = SX.sym('u2')  # Pitch command
    u3 = SX.sym('u3')  # Roll command
    u4 = SX.sym('u4')  # Thrust command
    u = vertcat(u1, u2, u3, u4)

    # Model parameters
    tp = SX.sym('tp')    # Roll time constant
    kp = SX.sym('kp')    # Roll gain
    tr = SX.sym('tr')    # Pitch time constant
    kr = SX.sym('kr')    # Pitch gain
    tyr = SX.sym('tr')   # Yaw rate time constant
    kyr = SX.sym('kr')   # Yaw rate gain
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
                tyr, kyr,
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
    qr_dot = SX.sym('qr_dot')
    qp_dot = SX.sym('qp_dot')
    qy_dot = SX.sym('qy_dot')
    qyr_dot = SX.sym('qyr_dot')
    x_dot = vertcat(px_dot, py_dot, pz_dot,
                    vx_dot, vy_dot, vz_dot,
                    qr_dot, qp_dot, qy_dot,
                    qyr_dot)

    # Dynamics
    dpx = vx
    dpy = vy
    dpz = vz
    dvx = dx * vx + (cos(qy)*sin(qp)*cos(qr) + sin(qy)*sin(qr)) * kth * u4 + fdx
    dvy = dy * vy + (sin(qy)*sin(qp)*cos(qr) - cos(qy)*sin(qr)) * kth * u4 + fdy
    dvz = dz * vz + (cos(qp)*cos(qr)) * kth * u4 + g + fdz
    dqr = (kr * u3 - qr) / tr
    dqp = (kp * u2 - qp) / tp
    dqy = qyr
    dqyr = (kyr * u3 - qyr) / tyr

    f_expl = vertcat(dpx, dpy, dpz,
                     dvx, dvy, dvz,
                     dqr, dqp, dqy,
                     dqyr)
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
