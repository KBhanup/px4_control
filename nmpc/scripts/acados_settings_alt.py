from acados_template import AcadosOcp, AcadosOcpSolver
from drone_model_alt import drone_model
import numpy as np


def acados_settings(Tf, N, generate_code=False):
    """
      Set's up the acados Optimal Control Problem and Solver

      ...

      Parameters
      ----------
      Tf : double
        Prediction horizon
      N : int
        Prediction steps
    """

    # Get drone model
    model = drone_model()

    # acados OCP handle
    ocp = AcadosOcp()

    # OCP dimensions
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ocp.dims.N = N

    # OCP costs
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    # State map
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    # Input map
    Vu = np.zeros((ny, nu))
    Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    # Terminal cost map
    Vx_e = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    # State and input cost
    W = np.diag([20, 20, 20,  # Position
                 5, 5, 5,     # Velocity
                 1, 1, 1, 1,  # Attitude
                 10,          # Yaw rate
                 150,         # Pitch
                 150,         # Roll
                 150])        # Thrust

    # Stage cost
    ocp.cost.W = W

    # Terminal cost
    ocp.cost.W_e = 10 * N * W[:nx, :nx]

    # References
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # OCP constraints
    # Attitude constraints
    q_min = -1.0
    q_max = 1.0

    ocp.constraints.idxbx = np.array([6, 7, 8, 9])
    ocp.constraints.lbx = np.array([q_min, q_min, q_min, q_min])
    ocp.constraints.ubx = np.array([q_max, q_max, q_max, q_max])

    # Input constraints
    yaw_rate_min = -0.1*np.pi
    yaw_rate_max = 0.1*np.pi
    pitch_cmd_min = -0.05*np.pi
    pitch_cmd_max = 0.05*np.pi
    roll_cmd_min = -0.05*np.pi
    roll_cmd_max = 0.05*np.pi
    thrust_min = 0.1
    thrust_max = 1.0

    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    ocp.constraints.lbu = np.array(
        [yaw_rate_min, pitch_cmd_min, roll_cmd_min, thrust_min])
    ocp.constraints.ubu = np.array(
        [yaw_rate_max, pitch_cmd_max, roll_cmd_max, thrust_max])

    # Initial state
    ocp.constraints.x0 = np.zeros(nx)

    # Model
    ocp.model = model
    ocp.parameter_values = np.array(
        [0.1355,      # Roll time constant
         0.9883,      # Roll gain
         0.1357,      # Pitch time constant
         0.9955,      # Pitch gain
         -0.4154,     # Damping x
         -0.4154,     # Damping y
         -0.1379,     # Damping z
         0,           # Disturbance force x
         0,           # Disturbance force y
         0,           # Disturbance force z
         28.5371,     # Thrust coefficients
         -9.8066])    # Gravity

    # OCP options
    ocp.solver_options.tf = Tf
    # PARTIAL_CONDENSING_HPIPM
    # FULL_CONDENSING_HPIPM
    # FULL_CONDENSING_QPOASES
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    # ocp.solver_options.nlp_solver_max_iter = 1000

    # Path to acados include and lib directories
    # Uncomment if you want to build generated c code
    # ocp.acados_include_path = '../../acados/include'
    # ocp.acados_lib_path = '../../acados/lib'

    acados_solver = AcadosOcpSolver(ocp)

    return model, acados_solver


if __name__ == "__main__":
    T = 2  # sec
    control_rate = 20  # Hz
    acados_settings(T, T*control_rate, generate_code=True)
    print("Acados NMPC generated")
