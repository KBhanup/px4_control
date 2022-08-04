from acados_template import AcadosModel
from casadi import SX, vertcat, atan2, asin


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
    qw = SX.sym('qw')    # qw
    qx = SX.sym('qx')    # qx
    qy = SX.sym('qy')    # qy
    qz = SX.sym('qz')    # qz
    x = vertcat(px, py, pz,
                vx, vy, vz,
                qw, qx, qy, qz)

    # Input
    y_cmd = SX.sym('y_cmd')  # Yaw rate
    p_cmd = SX.sym('p_cmd')  # Pitch
    r_cmd = SX.sym('r_cmd')  # Roll
    t_cmd = SX.sym('t_cmd')  # Thrust
    u = vertcat(y_cmd, p_cmd, r_cmd, t_cmd)

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
    qw_dot = SX.sym('qw_dot')
    qx_dot = SX.sym('qx_dot')
    qy_dot = SX.sym('qy_dot')
    qz_dot = SX.sym('qz_dot')
    x_dot = vertcat(px_dot, py_dot, pz_dot,
                    vx_dot, vy_dot, vz_dot,
                    qw_dot, qx_dot, qy_dot, qz_dot)

    # Dynamics
    dpx = vx
    dpy = vy
    dpz = vz
    dvx = dx * vx + 2 * (qw*qy + qx*qz) * kth * t_cmd + fdx
    dvy = dy * vy + 2 * (qy*qz - qw*qx) * kth * t_cmd + fdy
    dvz = dz * vz + (1 - 2 * (qx*qx + qy*qy)) * kth * t_cmd + g + fdz

    wx = (kr * r_cmd - atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))) / tr
    wy = (kp * p_cmd - asin(2*(qw*qy - qz*qx))) / tp
    wz = y_cmd

    dqw = 0.5 * (-wx*qx - wy*qy - wz*qz)
    dqx = 0.5 * ( wx*qw + wy*qz - wz*qy)
    dqy = 0.5 * (-wx*qz + wy*qw + wz*qx)
    dqz = 0.5 * ( wx*qy - wy*qw + wz*qw)

    f_expl = vertcat(dpx, dpy, dpz,
                     dvx, dvy, dvz,
                     dqw, dqx, dqy, dqz)
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
