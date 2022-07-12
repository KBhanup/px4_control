#include "nmpc/acados_nmpc.h"

#include <iostream>

namespace px4_ctrl {
AcadosNMPC::AcadosNMPC() {
#ifdef TRACK_TIME
  total_ocp_time = 0.0;
  min_ocp_time = 1.0e9;
  max_ocp_time = 0.0;
  total_ocp_calls = 0;
#endif
}

AcadosNMPC::~AcadosNMPC() {
#ifdef TRACK_TIME
  std::cout << "Average ocp solving time: "
            << 1000 * (total_ocp_time / total_ocp_calls) << "ms\n";
  std::cout << "Maximum ocp solving time: " << 1000 * max_ocp_time << "ms\n";
  std::cout << "Minimum ocp solving time: " << 1000 * min_ocp_time << "ms\n";
#endif
  // Free Acados Solver
  int status = drone_w_disturbances_acados_free(acados_ocp_capsule);
  if (status) {
    std::cerr << "drone_w_disturbances_acados_free() returned status: "
              << status << "\n";
  }
  // Free Acados Solver Capsule
  status = drone_w_disturbances_acados_free_capsule(acados_ocp_capsule);
  if (status) {
    std::cerr << "drone_w_disturbances_acados_free_capsule() returned status: "
              << status << "\n";
  }
}

bool AcadosNMPC::initializeController(const model_parameters &model_params) {
  // Initiallize Acados solver with default shooting intervals
  acados_ocp_capsule = drone_w_disturbances_acados_create_capsule();
  int status = drone_w_disturbances_acados_create(acados_ocp_capsule);

  if (status) {
    std::cerr << "drone_w_disturbances_acados_create() returned status:"
              << status << "\n";
    return false;
  }

  // Initialize Acados variables
  nlp_config = drone_w_disturbances_acados_get_nlp_config(acados_ocp_capsule);
  nlp_dims = drone_w_disturbances_acados_get_nlp_dims(acados_ocp_capsule);
  nlp_in = drone_w_disturbances_acados_get_nlp_in(acados_ocp_capsule);
  nlp_out = drone_w_disturbances_acados_get_nlp_out(acados_ocp_capsule);
  nlp_solver = drone_w_disturbances_acados_get_nlp_solver(acados_ocp_capsule);
  nlp_opts = drone_w_disturbances_acados_get_nlp_opts(acados_ocp_capsule);

  // Initial conditions
  int idxbx0[DRONE_W_DISTURBANCES_NBX0];
  idxbx0[0] = 0;
  idxbx0[1] = 1;
  idxbx0[2] = 2;
  idxbx0[3] = 3;
  idxbx0[4] = 4;
  idxbx0[5] = 5;
  idxbx0[6] = 6;
  idxbx0[7] = 7;
  idxbx0[8] = 8;
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx",
                                idxbx0);

  // Set parameters
  acados_model_parameters = new double[DRONE_W_DISTURBANCES_NP];
  acados_model_parameters[0] = model_params.t_roll;     // Roll time constant
  acados_model_parameters[1] = model_params.k_roll;     // Roll gain
  acados_model_parameters[2] = model_params.t_pitch;    // Pitch time constant
  acados_model_parameters[3] = model_params.k_pitch;    // Pitch gain
  acados_model_parameters[4] = model_params.damp_x;     // Damping x
  acados_model_parameters[5] = model_params.damp_y;     // Damping y
  acados_model_parameters[6] = model_params.damp_z;     // Damping z
  acados_model_parameters[7] = 0.0;                     // Disturbance force x
  acados_model_parameters[8] = 0.0;                     // Disturbance force y
  acados_model_parameters[9] = 0.0;                     // Disturbance force z
  acados_model_parameters[10] = model_params.k_thrust;  // Thrust coefficients
  acados_model_parameters[11] = model_params.gravity;   // Gravity

  hover_thrust = -model_params.gravity / model_params.k_thrust;

  for (int i = 0; i <= DRONE_W_DISTURBANCES_N; i++)
    drone_w_disturbances_acados_update_params(acados_ocp_capsule, i,
                                              acados_model_parameters,
                                              DRONE_W_DISTURBANCES_NP);

  return true;
}

bool AcadosNMPC::setWeighingMatrix(const std::vector<double> &weights) {
  if (weights.size() == DRONE_W_DISTURBANCES_NY) {
    double W[DRONE_W_DISTURBANCES_NY * DRONE_W_DISTURBANCES_NY];
    for (int i = 0; i < DRONE_W_DISTURBANCES_NY * DRONE_W_DISTURBANCES_NY; i++)
      W[i] = 0.0;

    W[0 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[0];    // Position x
    W[1 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[1];    // Position y
    W[2 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[2];    // Position z
    W[3 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[3];    // Velocity x
    W[4 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[4];    // Velocity y
    W[5 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[5];    // Velocity z
    W[6 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[6];    // Roll
    W[7 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[7];    // Pitch
    W[8 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[8];    // Yaw
    W[9 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[9];    // Yaw rate
    W[10 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[10];  // Pitch cmd
    W[11 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[11];  // Roll cmd
    W[12 * (DRONE_W_DISTURBANCES_NY + 1)] = weights[12];  // Thrust

    for (int i = 0; i < DRONE_W_DISTURBANCES_N; i++)
      ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);

    double WN[DRONE_W_DISTURBANCES_NX * DRONE_W_DISTURBANCES_NX];
    for (int i = 0; i < DRONE_W_DISTURBANCES_NX * DRONE_W_DISTURBANCES_NX; i++)
      WN[i] = 0.0;

    WN[0 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[0];
    WN[1 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[1];
    WN[2 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[2];
    WN[3 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[3];
    WN[4 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[4];
    WN[5 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[5];
    WN[6 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[6];
    WN[7 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[7];
    WN[8 * (DRONE_W_DISTURBANCES_NX + 1)] =
        10.0 * DRONE_W_DISTURBANCES_N * weights[8];

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, DRONE_W_DISTURBANCES_N,
                           "W", WN);
    return true;
  } else {
    std::cout << "Was expecting a vector of size " << DRONE_W_DISTURBANCES_NY
              << "\n";
  }
  return false;
}

void AcadosNMPC::setTrajectory(
    const std::vector<trajectory_setpoint> &trajectory) {
  current_reference_trajectory.clear();
  current_reference_trajectory = trajectory;
  trajectory_index = 0;
  trajectory_length = current_reference_trajectory.size();
}

bool AcadosNMPC::getTrajectory(std::vector<trajectory_setpoint> &trajectory,
                               const trajectory_setpoint &start_point,
                               const trajectory_setpoint &goal_point,
                               const std::vector<double> &disturbances) {
  // Use goal point as the reference trajectory
  current_reference_trajectory.clear();
  current_reference_trajectory.push_back(goal_point);
  trajectory_index = 0;
  trajectory_length = current_reference_trajectory.size();

  // Clean trajectory handle, set goal point as the reference
  trajectory.clear();
  updateReference();

  // Initialize state
  setCurrentState(start_point, disturbances);
  bool reached_goal = false;

  // Run NMPC to create trajectory
  while (!reached_goal) {
    int status = drone_w_disturbances_acados_solve(acados_ocp_capsule);

    if (status == ACADOS_SUCCESS) {
      // Get state at each time step
      double u_i[DRONE_W_DISTURBANCES_NU];
      double x_i[DRONE_W_DISTURBANCES_NX];
      for (int i = 0; i < DRONE_W_DISTURBANCES_N; i++) {
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", &u_i);
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", &x_i);

        // Naive way to see if we reached the goal
        double sum_u = std::abs(u_i[0]) + std::abs(u_i[1]) + std::abs(u_i[2]) +
                       std::abs(u_i[3] - hover_thrust);
        double dp = std::abs(goal_point.pos_x - x_i[0]) +
                    std::abs(goal_point.pos_y - x_i[1]) +
                    std::abs(goal_point.pos_z - x_i[2]);

        // If we are not close add point
        if (dp > 0.05 || sum_u > 0.05) {
          trajectory_setpoint setpoint;
          setpoint.pos_x = x_i[0];
          setpoint.pos_y = x_i[1];
          setpoint.pos_z = x_i[2];
          setpoint.vel_x = x_i[3];
          setpoint.vel_y = x_i[4];
          setpoint.vel_z = x_i[5];
          setpoint.q_roll = x_i[6];
          setpoint.q_pitch = x_i[7];
          setpoint.q_yaw = x_i[8];
          trajectory.push_back(setpoint);
        } else {
          trajectory.push_back(goal_point);
          reached_goal = true;
          break;
        }

        // Trajectory just got longer than 5 minutes
        if (trajectory.size() > 6000) {
          std::cerr << "Looks like it's getting out of hand\n";
          return false;
        }
      }
      // In case we haven't reached the goal yet update the current state and
      // run the nmpc again
      if (!reached_goal) {
        trajectory_setpoint setpoint;
        setpoint.pos_x = x_i[0];
        setpoint.pos_y = x_i[1];
        setpoint.pos_z = x_i[2];
        setpoint.vel_x = x_i[3];
        setpoint.vel_y = x_i[4];
        setpoint.vel_z = x_i[5];
        setpoint.q_roll = x_i[6];
        setpoint.q_pitch = x_i[7];
        setpoint.q_yaw = x_i[8];
        setCurrentState(setpoint, disturbances);
      }
    } else {
      std::cerr << "drone_w_disturbances_acados_solve() failed with status: "
                << status << "\n";
      return false;
    }
  }
  return reached_goal;
}

void AcadosNMPC::setCurrentState(const trajectory_setpoint &state,
                                 const std::vector<double> &disturbances) {
  updateInitialConditions(state);
  updateDisturbances(disturbances[0], disturbances[1], disturbances[2]);
}

bool AcadosNMPC::getCommands(std::vector<double> &ctrl) {
  // Update reference
  updateReference();

  int status = drone_w_disturbances_acados_solve(acados_ocp_capsule);

#ifdef TRACK_TIME
  double elapsed_time;
  ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
  max_ocp_time = max_ocp_time > elapsed_time ? max_ocp_time : elapsed_time;
  min_ocp_time = min_ocp_time < elapsed_time ? min_ocp_time : elapsed_time;
  total_ocp_time += elapsed_time;
  total_ocp_calls++;
#endif

  // Get first control input
  if (status == ACADOS_SUCCESS) {
    double u_0[DRONE_W_DISTURBANCES_NU];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_0);
    ctrl.clear();
    ctrl.push_back(u_0[0]);
    ctrl.push_back(u_0[1]);
    ctrl.push_back(u_0[2]);
    ctrl.push_back(u_0[3]);

    // Update reference index
    trajectory_index = trajectory_index + 1 < trajectory_length
                           ? trajectory_index + 1
                           : trajectory_index;

    return true;
  } else {
    std::cerr << "drone_w_disturbances_acados_solve() failed with status: "
              << status << "\n";
  }
  return false;
}

void AcadosNMPC::updateReference() {
  int ref_index = trajectory_index;
  int acados_index = 0;

  // Trajectory
  while (acados_index < DRONE_W_DISTURBANCES_N) {
    double y_ref[DRONE_W_DISTURBANCES_NY];
    y_ref[0] = current_reference_trajectory[ref_index].pos_x;    // Position x
    y_ref[1] = current_reference_trajectory[ref_index].pos_y;    // Position y
    y_ref[2] = current_reference_trajectory[ref_index].pos_z;    // Position z
    y_ref[3] = current_reference_trajectory[ref_index].vel_x;    // Velocity x
    y_ref[4] = current_reference_trajectory[ref_index].vel_y;    // Velocity y
    y_ref[5] = current_reference_trajectory[ref_index].vel_z;    // Velocity z
    y_ref[6] = current_reference_trajectory[ref_index].q_roll;   // Roll
    y_ref[7] = current_reference_trajectory[ref_index].q_pitch;  // Pitch
    y_ref[8] = current_reference_trajectory[ref_index].q_yaw;    // Yaw
    y_ref[9] = 0.0;                                              // Yaw rate
    y_ref[10] = 0.0;                                             // Pitch cmd
    y_ref[11] = 0.0;                                             // Roll cmd
    y_ref[12] = hover_thrust;                                    // Thrust

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, acados_index, "yref",
                           y_ref);

    acados_index++;
    ref_index = ref_index + 1 < trajectory_length ? ref_index + 1 : ref_index;
  }

  // End point
  double y_ref_e[DRONE_W_DISTURBANCES_NYN];
  y_ref_e[0] = current_reference_trajectory[ref_index].pos_x;    // Position x
  y_ref_e[1] = current_reference_trajectory[ref_index].pos_y;    // Position y
  y_ref_e[2] = current_reference_trajectory[ref_index].pos_z;    // Position z
  y_ref_e[3] = current_reference_trajectory[ref_index].vel_x;    // Velocity x
  y_ref_e[4] = current_reference_trajectory[ref_index].vel_y;    // Velocity y
  y_ref_e[5] = current_reference_trajectory[ref_index].vel_z;    // Velocity z
  y_ref_e[6] = current_reference_trajectory[ref_index].q_roll;   // Roll
  y_ref_e[7] = current_reference_trajectory[ref_index].q_pitch;  // Pitch
  y_ref_e[8] = current_reference_trajectory[ref_index].q_yaw;    // Yaw

  ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, DRONE_W_DISTURBANCES_N,
                         "yref", y_ref_e);
}

void AcadosNMPC::updateInitialConditions(
    const trajectory_setpoint &state_init) {
  double x_init[DRONE_W_DISTURBANCES_NBX0];
  x_init[0] = state_init.pos_x;
  x_init[1] = state_init.pos_y;
  x_init[2] = state_init.pos_z;
  x_init[3] = state_init.vel_x;
  x_init[4] = state_init.vel_y;
  x_init[5] = state_init.vel_z;
  x_init[6] = state_init.q_roll;
  x_init[7] = state_init.q_pitch;
  x_init[8] = state_init.q_yaw;

  // Set the initial conditions by setting the lower and upper bounds to the
  // initial state values
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_init);
  ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_init);

  double u0[DRONE_W_DISTURBANCES_NU];
  u0[0] = 0.0;
  u0[1] = 0.0;
  u0[2] = 0.0;
  u0[3] = hover_thrust;

  for (int i = 0; i <= DRONE_W_DISTURBANCES_N; i++) {
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
  }
}

void AcadosNMPC::updateDisturbances(const double &fdis_x, const double &fdis_y,
                                    const double &fdis_z) {
  acados_model_parameters[7] = fdis_x;
  acados_model_parameters[8] = fdis_y;
  acados_model_parameters[9] = fdis_z;

  for (int i = 0; i <= DRONE_W_DISTURBANCES_N; i++)
    drone_w_disturbances_acados_update_params(acados_ocp_capsule, i,
                                              acados_model_parameters,
                                              DRONE_W_DISTURBANCES_NP);
}

}  // namespace px4_ctrl