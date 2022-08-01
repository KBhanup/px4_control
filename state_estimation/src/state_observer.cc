#include "state_estimation/state_observer.h"

#include <future>

// ROS
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_datatypes.h"

namespace px4_ctrl {
StateObserver::StateObserver(ros::NodeHandle &nh) {
  // Setup Subscribers
  pose_sub = nh.subscribe("/mavros/local_position/odom", 1,
                          &StateObserver::poseCallback, this);
  odom_sub = nh.subscribe("/mavros/local_position/odom", 1,
                          &StateObserver::odomCallback, this);
  marker_sub =
      nh.subscribe("/marker/pose", 1, &StateObserver::markerCallback, this);
  att_ctrl_sub = nh.subscribe("/mavros/setpoint_raw/attitude", 1,
                              &StateObserver::attCtrlCallback, this);
  mavros_status_sub = nh.subscribe("/mavros/state", 1,
                                   &StateObserver::mavrosStatusCallback, this);
  mission_state_sub = nh.subscribe("/mission_state", 1,
                                   &StateObserver::missionStateCallback, this);

  // Setup Publisher
  state_pub =
      nh.advertise<px4_control_msgs::DroneStateMarker>("/drone_state", 1);

  // Initialize Parameters
  loadParameters();
  is_initialized = false;
  marker_found = false;
  wt_sensor = true;
  k_thrust = k_thrust_wt_sensor;

  // Initialize Sensors
  pose_sensor = new PoseSensor(R_pose, 50);
  odom_sensor = new OdometrySensor(R_odom, 30);
  marker_sensor = new MarkerSensor(R_marker, 10);

  /** TODO: Not sure about the initialization of the time stamp */
  past_cmd = Eigen::Vector4d(0.0, 0.0, 0.0, -gravity / k_thrust);
  latest_cmd = Eigen::Vector4d(0.0, 0.0, 0.0, -gravity / k_thrust);
  latest_cmd_time = ros::Time::now();
}

StateObserver::~StateObserver() {
  std::cout << "Pose sensor last R = \n"
            << pose_sensor->getCurrentR() << "\n\n";

  std::cout << "Odometry sensor last R = \n"
            << odom_sensor->getCurrentR() << "\n\n";

  std::cout << "Marker sensor last R = \n"
            << marker_sensor->getCurrentR() << "\n\n";

  delete pose_sensor;
  delete odom_sensor;
  delete marker_sensor;
}

void StateObserver::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  // Drone model parameters
  nh_pvt.param("t_pitch", t_pitch, 1.0);
  nh_pvt.param("k_pitch", k_pitch, 1.0);

  nh_pvt.param("t_roll", t_roll, 1.0);
  nh_pvt.param("k_roll", k_roll, 1.0);

  nh_pvt.param("damping_x", damp_x, -1.0);
  nh_pvt.param("damping_y", damp_y, -1.0);
  nh_pvt.param("damping_z", damp_z, -1.0);
  damping_matrix << damp_x, 0.0, 0.0, 0.0, damp_y, 0.0, 0.0, 0.0, damp_z;

  nh_pvt.param("k_thrust_wt_sensor", k_thrust_wt_sensor, 1.0);
  nh_pvt.param("k_thrust_wo_sensor", k_thrust_wo_sensor, 1.0);

  nh_pvt.param("gravity", gravity, -9.8066);
  gravity_vector << 0.0, 0.0, gravity;

  nh_pvt.param("disturbance_limit", disturbance_limit, 10.0);

  // Observer parameters
  // Error covariance
  std::vector<double> P_p, P_v, P_dq, P_fd, P_rb, P_mp, P_mdq;
  nh_pvt.getParam("P_p", P_p);
  nh_pvt.getParam("P_v", P_v);
  nh_pvt.getParam("P_dq", P_dq);
  nh_pvt.getParam("P_fd", P_fd);
  nh_pvt.getParam("P_rb", P_rb);
  nh_pvt.getParam("P_mp", P_mp);
  nh_pvt.getParam("P_mdq", P_mdq);

  // Process covariance
  double Q_p, Q_v, Q_dq, Q_fd, Q_rb, Q_mp, Q_mdq;
  nh_pvt.param("Q_p", Q_p, 0.1);
  nh_pvt.param("Q_v", Q_v, 0.1);
  nh_pvt.param("Q_dq", Q_dq, 0.1);
  nh_pvt.param("Q_fd", Q_fd, 0.1);
  nh_pvt.param("Q_rb", Q_rb, 0.1);
  nh_pvt.param("Q_mp", Q_mp, 0.001);
  nh_pvt.param("Q_mdq", Q_mdq, 0.001);

  // Pose covariance
  std::vector<double> R_pose_p, R_pose_q;
  nh_pvt.getParam("R_pose_p", R_pose_p);
  nh_pvt.getParam("R_pose_q", R_pose_q);

  // Odometry covariance
  std::vector<double> R_odom_v;
  nh_pvt.getParam("R_odom_v", R_odom_v);

  // Marker covariance
  std::vector<double> R_marker_p, R_marker_q;
  nh_pvt.getParam("R_marker_p", R_marker_p);
  nh_pvt.getParam("R_marker_q", R_marker_q);

  // Chi critical values
  nh_pvt.param("pose_chi_critical", pose_chi_critical, 18.548);
  nh_pvt.param("odom_chi_critical", odom_chi_critical, 12.838);
  nh_pvt.param("marker_chi_critical", marker_chi_critical, 18.548);

  // Initialize matrices
  P_mat.setZero();
  P_mat.block(0, 0, 3, 3).diagonal() = Eigen::Vector3d(P_p[0], P_p[1], P_p[2]);
  P_mat.block(3, 3, 3, 3).diagonal() = Eigen::Vector3d(P_v[0], P_v[1], P_v[2]);
  P_mat.block(6, 6, 3, 3).diagonal() =
      Eigen::Vector3d(P_dq[0], P_dq[1], P_dq[2]);
  P_mat.block(9, 9, 3, 3).diagonal() =
      Eigen::Vector3d(P_fd[0], P_fd[1], P_fd[2]);
  P_mat.block(12, 12, 3, 3).diagonal() =
      Eigen::Vector3d(P_rb[0], P_rb[1], P_rb[2]);
  P_mat.block(15, 15, 3, 3).diagonal() =
      Eigen::Vector3d(P_mp[0], P_mp[1], P_mp[2]);
  P_mat.block(18, 18, 3, 3).diagonal() =
      Eigen::Vector3d(P_mdq[0], P_mdq[1], P_mdq[2]);

  Q_mat.setZero();
  Q_mat.block(0, 0, 3, 3) = Q_p * Eigen::Matrix3d::Identity();
  Q_mat.block(3, 3, 3, 3) = Q_v * Eigen::Matrix3d::Identity();
  Q_mat.block(6, 6, 3, 3) = Q_dq * Eigen::Matrix3d::Identity();
  Q_mat.block(9, 9, 3, 3) = Q_fd * Eigen::Matrix3d::Identity();
  Q_mat.block(12, 12, 3, 3) = Q_rb * Eigen::Matrix3d::Identity();
  Q_mat.block(15, 15, 3, 3) = Q_mp * Eigen::Matrix3d::Identity();
  Q_mat.block(18, 18, 3, 3) = Q_mdq * Eigen::Matrix3d::Identity();

  R_odom.setZero();
  R_odom.block(0, 0, 3, 3).diagonal() =
      Eigen::Vector3d(R_odom_v[0], R_odom_v[1], R_odom_v[2]);

  R_pose.setZero();
  R_pose.block(0, 0, 3, 3).diagonal() =
      Eigen::Vector3d(R_pose_p[0], R_pose_p[1], R_pose_p[2]);
  R_pose.block(3, 3, 4, 4).diagonal() =
      Eigen::Vector4d(R_pose_q[0], R_pose_q[1], R_pose_q[2], R_pose_q[3]);

  R_marker.setZero();
  R_marker.block(0, 0, 3, 3).diagonal() =
      Eigen::Vector3d(R_marker_p[0], R_marker_p[1], R_marker_p[2]);
  R_marker.block(3, 3, 4, 4).diagonal() = Eigen::Vector4d(
      R_marker_q[0], R_marker_q[1], R_marker_q[2], R_marker_q[3]);
}

// Attitude Control Callback
void StateObserver::attCtrlCallback(const mavros_msgs::AttitudeTarget &msg) {
  // Get Pitch and Roll commands from the quaternion
  tf::Quaternion q_cmd;
  tf::quaternionMsgToTF(msg.orientation, q_cmd);
  double y_cmd, p_cmd, r_cmd;
  tf::Matrix3x3(q_cmd).getEulerYPR(y_cmd, p_cmd, r_cmd);

  // Construct command
  Eigen::Vector4d cmd(msg.body_rate.z, p_cmd, r_cmd, (double)msg.thrust);

  // Update commands
  // I need to set past_cmd == cmd because of the weird time stamps
  if ((msg.header.stamp - latest_cmd_time).toSec() < 0.5)
    past_cmd = latest_cmd;
  else
    past_cmd = cmd;

  latest_cmd = cmd;
  latest_cmd_time = msg.header.stamp;
}

// Status Callback
void StateObserver::mavrosStatusCallback(
    const mavros_msgs::State::ConstPtr &msg) {
  current_status = *msg;
}

// Pose Callback
void StateObserver::poseCallback(const nav_msgs::Odometry &msg) {
  // Initialize state with the first pose message
  if (!is_initialized) {
    // Time
    past_state_time = msg.header.stamp;

    // Position
    state.position(0) = msg.pose.pose.position.x;
    state.position(1) = msg.pose.pose.position.y;
    state.position(2) = msg.pose.pose.position.z;

    // Attitude
    state.attitude = Eigen::Quaterniond(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

    // Velocity
    state.velocity.setZero();

    // Disturbances
    state.disturbances.setZero();

    // Biases
    state.random_walk_bias.setZero();

    last_drone_q = state.attitude;

    is_initialized = true;
  } else {
    double dt = (msg.header.stamp - past_state_time).toSec();
    // If there are big gaps in the pose message re-initialize keeping the old
    // error covariance
    if (dt > 0.5) {
      ROS_WARN("Large gap detected. Re-initializing");
      // Time
      past_state_time = msg.header.stamp;

      // Position
      state.position(0) = msg.pose.pose.position.x;
      state.position(1) = msg.pose.pose.position.y;
      state.position(2) = msg.pose.pose.position.z;

      // Attitude
      state.attitude = Eigen::Quaterniond(
          msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

      // Velocity
      state.velocity.setZero();

      last_drone_q = state.attitude;

      // Use the same error covariance but increase it
      P_mat = P_mat + 30 * dt * Q_mat;
    } else {
      // Prepare measurement
      Eigen::Quaterniond q_meas(
          msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
      Eigen::Quaterniond dq = last_drone_q.inverse() * q_meas;

      if (dq.w() < 0) {
        q_meas.w() = -q_meas.w();
        q_meas.x() = -q_meas.x();
        q_meas.y() = -q_meas.y();
        q_meas.z() = -q_meas.z();
      }

      last_drone_q = q_meas;

      Eigen::Matrix<double, pose_size, 1> y_pose;
      y_pose << msg.pose.pose.position.x, msg.pose.pose.position.y,
          msg.pose.pose.position.z, q_meas.w(), q_meas.x(), q_meas.y(),
          q_meas.z();

      // Run prediction and update time
      predict(msg.header.stamp);

      // Run correction
      Eigen::MatrixXd H_mat;
      Eigen::MatrixXd y_expected;
      pose_sensor->correctionData(state_pred, H_mat, y_expected);

      Eigen::MatrixXd P_y = H_mat * P_pred_mat * H_mat.transpose();
      Eigen::MatrixXd S_inv = (P_y + pose_sensor->getCurrentR()).inverse();
      Eigen::MatrixXd innovation = y_pose - y_expected;

      // Run sensor covariance update
      pose_sensor->updateR(innovation, P_y);

      // Measurement gating
      double e_squared = (innovation.transpose() * S_inv * innovation).sum();

      if (e_squared < pose_chi_critical) {
        Eigen::MatrixXd K_mat = P_pred_mat * H_mat.transpose() * S_inv;

        error_state = K_mat * innovation;

        // Update state
        std::future<void> state_update_f =
            std::async(std::launch::async, &StateObserver::correctState, this);

        // Update P marix and make sure it's symmetric
        Eigen::MatrixXd IKH_mat =
            Eigen::MatrixXd::Identity(error_state_size, error_state_size) -
            K_mat * H_mat;
        P_mat = IKH_mat * P_pred_mat;
        P_mat = 0.5 * (P_mat + P_mat.transpose());

        // Publish state
        state_update_f.wait();
      } else {
        ROS_WARN("Pose measurement was rejected");
      }
      publishState(msg.header.stamp);
    }
  }
}

// Odometry Callback
void StateObserver::odomCallback(const nav_msgs::Odometry &msg) {
  if (is_initialized) {
    // Prepare measurement
    Eigen::Matrix<double, odom_size, 1> y_odom;
    y_odom << msg.twist.twist.linear.x, msg.twist.twist.linear.y,
        msg.twist.twist.linear.z;

    // Run prediction and update time
    predict(msg.header.stamp);

    // Run correction
    Eigen::MatrixXd H_mat;
    Eigen::MatrixXd y_expected;
    odom_sensor->correctionData(state_pred, H_mat, y_expected);

    Eigen::MatrixXd P_y = H_mat * P_pred_mat * H_mat.transpose();
    Eigen::MatrixXd S_inv = (P_y + odom_sensor->getCurrentR()).inverse();
    Eigen::MatrixXd innovation = y_odom - y_expected;

    // Run sensor covariance update
    odom_sensor->updateR(innovation, P_y);

    // Measurement gating
    double e_squared = (innovation.transpose() * S_inv * innovation).sum();

    if (e_squared < odom_chi_critical) {
      Eigen::MatrixXd K_mat = P_pred_mat * H_mat.transpose() * S_inv;

      error_state = K_mat * innovation;

      // Update state
      std::future<void> state_update_f =
          std::async(std::launch::async, &StateObserver::correctState, this);

      // Update P marix and make sure it's symmetric
      Eigen::MatrixXd IKH_mat =
          Eigen::MatrixXd::Identity(error_state_size, error_state_size) -
          K_mat * H_mat;
      P_mat = IKH_mat * P_pred_mat;
      P_mat = 0.5 * (P_mat + P_mat.transpose());

      // Publish state
      state_update_f.wait();
    } else {
      ROS_WARN("Odometry measurement was rejected");
    }
  }
}

// Marker callback
void StateObserver::markerCallback(const geometry_msgs::PoseStamped &msg) {
  if (is_initialized) {
    if (!marker_found) {
      // Initialize marker position
      Eigen::Quaterniond q_meas(msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z);
      Eigen::Vector3d p_meas(msg.pose.position.x, msg.pose.position.y,
                             msg.pose.position.z);

      state.marker_orientation = state.attitude * q_meas;
      state.marker_position =
          state.position + state.attitude.toRotationMatrix() * p_meas;

      marker_found = true;

      last_marker_q = q_meas;
    } else {
      // Prepare measurement
      Eigen::Quaterniond q_meas(msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z);
      Eigen::Quaterniond dq = last_marker_q.inverse() * q_meas;

      if (dq.w() < 0) {
        q_meas.w() = -q_meas.w();
        q_meas.x() = -q_meas.x();
        q_meas.y() = -q_meas.y();
        q_meas.z() = -q_meas.z();
      }

      last_marker_q = q_meas;

      Eigen::Matrix<double, marker_size, 1> y_marker;
      y_marker << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
          q_meas.w(), q_meas.x(), q_meas.y(), q_meas.z();

      // Run prediction and update time
      predict(msg.header.stamp);

      // Run correction
      Eigen::MatrixXd H_mat;
      Eigen::MatrixXd y_expected;
      marker_sensor->correctionData(state_pred, H_mat, y_expected);

      Eigen::MatrixXd P_y = H_mat * P_pred_mat * H_mat.transpose();
      Eigen::MatrixXd S_inv = (P_y + marker_sensor->getCurrentR()).inverse();
      Eigen::MatrixXd innovation = y_marker - y_expected;

      // Run sensor covariance update
      marker_sensor->updateR(innovation, P_y);

      // Measurement gating
      double e_squared = (innovation.transpose() * S_inv * innovation).sum();

      if (e_squared < marker_chi_critical) {
        Eigen::MatrixXd K_mat = P_pred_mat * H_mat.transpose() * S_inv;

        error_state = K_mat * innovation;
        // Zero out any change to the drone's pitch and roll that might
        // affect the drone's stability
        error_state(6) = 0.0;
        error_state(7) = 0.0;

        // Update state
        std::future<void> state_update_f =
            std::async(std::launch::async, &StateObserver::correctState, this);

        // Update P marix and make sure it's symmetric
        Eigen::MatrixXd IKH_mat =
            Eigen::MatrixXd::Identity(error_state_size, error_state_size) -
            K_mat * H_mat;
        P_mat = IKH_mat * P_pred_mat;
        P_mat = 0.5 * (P_mat + P_mat.transpose());

        // Publish state
        state_update_f.wait();
      } else {
        ROS_WARN("Marker measurement was rejected");
      }
    }
  }
}

void StateObserver::missionStateCallback(
    const px4_control_msgs::MissionState &msg) {
  bool wt_sensor_new = msg.wt_sensor.data;

  if (wt_sensor_new != wt_sensor) {
    if (wt_sensor_new)
      k_thrust = k_thrust_wt_sensor;
    else
      k_thrust = k_thrust_wo_sensor;
    wt_sensor = wt_sensor_new;
  }
}

void StateObserver::predict(ros::Time pred_time) {
  double dt = (pred_time - past_state_time).toSec();

  /** TODO: There's a chance that the marker arrives with a stamp prior to the
   * latest odometry thus making the dt negative. In this case skip prediction.
   * Using a circular buffer and rerun the filter will be a much better
   * solution*/
  if (dt > 0.0) {
    // Find input for the prediction time step
    // If we have recent attitude commands use full model
    if (current_status.mode == "OFFBOARD" &&
        ((pred_time - latest_cmd_time).toSec() < 0.5)) {
      Eigen::Vector4d cmd = Eigen::Vector4d::Zero();
      double dt_l = clipValue((pred_time - latest_cmd_time).toSec(), 0.0, dt);
      double dt_p = dt_l < dt ? dt - dt_l : 0.0;

      cmd = (dt_p / dt) * past_cmd + (dt_l / dt) * latest_cmd;

      // Run error covariance prediction asynchronously
      std::future<void> p_pred_update_f =
          std::async(std::launch::async, &StateObserver::updatePpred, this,
                     std::cref(dt), std::cref(cmd));

      // Use Runge Kutta 4 to propagate state
      Eigen::VectorXd k1 = getSystemDerivative(state, cmd);
      Eigen::VectorXd k2 =
          getSystemDerivative(addUpdate(state, 0.5 * dt * k1), cmd);
      Eigen::VectorXd k3 =
          getSystemDerivative(addUpdate(state, 0.5 * dt * k2), cmd);
      Eigen::VectorXd k4 = getSystemDerivative(addUpdate(state, dt * k3), cmd);

      state_pred = addUpdate(state, dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6);
    }
    // If we don't have recent attitude commands use simple model
    else {
      // Propagate state
      state_pred.position = state.position + dt * state.velocity;
      state_pred.velocity =
          state.velocity + dt * damping_matrix * state.velocity;
      state_pred.attitude = state.attitude;
      state_pred.disturbances.setZero();
      state_pred.random_walk_bias = state.random_walk_bias;
      state_pred.marker_position = state.marker_position;
      state_pred.marker_orientation = state.marker_orientation;

      // Construct F matrix
      Eigen::Matrix<double, error_state_size, error_state_size> F_mat;
      F_mat.setZero();

      // Position
      F_mat.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
      F_mat.block(0, 3, 3, 3) =
          dt * Eigen::Matrix3d::Identity() + 0.5 * dt * dt * damping_matrix;

      // Velocity
      F_mat.block(3, 3, 3, 3) =
          Eigen::Matrix3d::Identity() + dt * damping_matrix;

      // Attitude
      F_mat.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();

      // Disturbances
      F_mat.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity();

      // Random walk bias
      F_mat.block(12, 12, 3, 3) = Eigen::Matrix3d::Identity();

      // Marker position
      F_mat.block(15, 15, 3, 3) = Eigen::Matrix3d::Identity();

      // Marker orientation
      F_mat.block(18, 18, 3, 3) = Eigen::Matrix3d::Identity();

      // Update P_pred
      P_pred_mat = F_mat * P_mat * F_mat.transpose() + Q_mat;
    }
    past_state_time = pred_time;
  } else {
    state_pred = state;
    P_pred_mat = P_mat + Q_mat;
  }
}

Eigen::VectorXd StateObserver::getSystemDerivative(const eskf_state &state,
                                                   const Eigen::Vector4d &cmd) {
  // Get Rotation matrix
  Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();

  // Thrust
  Eigen::Vector3d T(0.0, 0.0, k_thrust * cmd(3));
  Eigen::Vector3d T_w = R_mat * T;

  // Damping
  Eigen::Vector3d D = damping_matrix * state.velocity;

  // Initialize derivative
  Eigen::VectorXd systemDerivative(10);
  systemDerivative.setZero();

  // Position derivative
  systemDerivative.segment(0, 3) = state.velocity;

  // Velocity derivative
  systemDerivative.segment(3, 3) =
      D + T_w + gravity_vector + state.disturbances;

  // Omegas
  double pitch = asin(R_mat(2, 0));
  double roll = atan2(R_mat(2, 1), R_mat(2, 2));

  double omega_z = cmd(0);
  double omega_y = (k_pitch * cmd(1) - pitch) / t_pitch;
  double omega_x = (k_roll * cmd(2) - roll) / t_roll;

  // Attitude derivative
  systemDerivative(6) =
      0.5 * (-omega_x * state.attitude.x() - omega_y * state.attitude.y() -
             omega_z * state.attitude.z());
  systemDerivative(7) =
      0.5 * (omega_x * state.attitude.w() + omega_y * state.attitude.z() -
             omega_z * state.attitude.y());
  systemDerivative(8) =
      0.5 * (-omega_x * state.attitude.z() + omega_y * state.attitude.w() +
             omega_z * state.attitude.x());
  systemDerivative(9) =
      0.5 * (omega_x * state.attitude.y() - omega_y * state.attitude.w() +
             omega_z * state.attitude.w());

  return systemDerivative;
}

eskf_state StateObserver::addUpdate(const eskf_state &state,
                                    const Eigen::VectorXd &state_update) {
  eskf_state updated_state;
  updated_state.position = state.position + state_update.segment(0, 3);
  updated_state.velocity = state.velocity + state_update.segment(3, 3);
  updated_state.attitude =
      Eigen::Quaterniond(state.attitude.w() + state_update(6),
                         state.attitude.x() + state_update(7),
                         state.attitude.y() + state_update(8),
                         state.attitude.z() + state_update(9));
  updated_state.attitude.normalize();
  updated_state.disturbances = state.disturbances;
  updated_state.random_walk_bias = state.random_walk_bias;
  updated_state.marker_position = state.marker_position;
  updated_state.marker_orientation = state.marker_orientation;

  return updated_state;
}

void StateObserver::updatePpred(const double &dt, const Eigen::Vector4d &cmd) {
  // Construct F matrix
  Eigen::Matrix<double, error_state_size, error_state_size> F_mat;
  F_mat.setZero();

  Eigen::Matrix3d R_mat = state.attitude.toRotationMatrix();
  Eigen::Vector3d T(0.0, 0.0, k_thrust * cmd(3));
  Eigen::Matrix3d T_w = R_mat * toSkew(T);

  // Position
  F_mat.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
  F_mat.block(0, 3, 3, 3) =
      dt * Eigen::Matrix3d::Identity() + 0.5 * dt * dt * damping_matrix;
  F_mat.block(0, 6, 3, 3) = -0.5 * dt * dt * T_w;
  F_mat.block(0, 9, 3, 3) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();

  // Velocity
  F_mat.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() + dt * damping_matrix;
  F_mat.block(3, 6, 3, 3) = -dt * T_w;
  F_mat.block(3, 9, 3, 3) = dt * Eigen::Matrix3d::Identity();

  // Omegas
  double pitch = asin(R_mat(2, 0));
  double roll = atan2(R_mat(2, 1), R_mat(2, 2));

  double omega_z = cmd(0);
  double omega_y = (k_pitch * cmd(1) - pitch) / t_pitch;
  double omega_x = (k_roll * cmd(2) - roll) / t_roll;

  // Attitude
  F_mat.block(6, 6, 3, 3) =
      eulerToRotMat(dt * omega_z, dt * omega_y, dt * omega_x);

  // Disturbances
  F_mat.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity();

  // Random walk bias
  F_mat.block(12, 12, 3, 3) = Eigen::Matrix3d::Identity();

  // Marker position
  F_mat.block(15, 15, 3, 3) = Eigen::Matrix3d::Identity();

  // Marker orientation
  F_mat.block(18, 18, 3, 3) = Eigen::Matrix3d::Identity();

  // Update P_pred
  P_pred_mat = F_mat * P_mat * F_mat.transpose() + Q_mat;
}

void StateObserver::correctState() {
  // Position
  state.position(0) = state_pred.position(0) + error_state(0);
  state.position(1) = state_pred.position(1) + error_state(1);
  state.position(2) = state_pred.position(2) + error_state(2);

  // Velocity
  state.velocity(0) = state_pred.velocity(0) + error_state(3);
  state.velocity(1) = state_pred.velocity(1) + error_state(4);
  state.velocity(2) = state_pred.velocity(2) + error_state(5);

  // Attitude
  Eigen::Vector3d thetas;
  thetas << error_state(6), error_state(7), error_state(8);
  double angle = thetas.norm();
  if (angle > 0) {
    thetas = thetas / angle;
    Eigen::Quaterniond dq =
        Eigen::Quaterniond(Eigen::AngleAxisd(angle, thetas));
    state.attitude = state_pred.attitude * dq;
  } else {
    state.attitude = state_pred.attitude;
  }

  // Disturbances
  state.disturbances(0) = clipValue(state_pred.disturbances(0) + error_state(9),
                                    -disturbance_limit, disturbance_limit);
  state.disturbances(1) =
      clipValue(state_pred.disturbances(1) + error_state(10),
                -disturbance_limit, disturbance_limit);
  state.disturbances(2) =
      clipValue(state_pred.disturbances(2) + error_state(11),
                -disturbance_limit, disturbance_limit);

  // Random walk bias
  state.random_walk_bias(0) = state_pred.random_walk_bias(0) + error_state(12);
  state.random_walk_bias(1) = state_pred.random_walk_bias(1) + error_state(13);
  state.random_walk_bias(2) = state_pred.random_walk_bias(2) + error_state(14);

  if (marker_found) {
    // Marker position
    state.marker_position(0) = state_pred.marker_position(0) + error_state(15);
    state.marker_position(1) = state_pred.marker_position(1) + error_state(16);
    state.marker_position(2) = state_pred.marker_position(2) + error_state(17);

    // Marker orientation
    Eigen::Vector3d thetas_marker;
    thetas_marker << error_state(18), error_state(19), error_state(20);
    angle = thetas_marker.norm();
    if (angle > 0) {
      thetas_marker = thetas_marker / angle;
      Eigen::Quaterniond dq_marker =
          Eigen::Quaterniond(Eigen::AngleAxisd(angle, thetas_marker));
      state.marker_orientation = state_pred.marker_orientation * dq_marker;
    } else {
      state.marker_orientation = state_pred.marker_orientation;
    }
  }
}

void StateObserver::publishState(ros::Time time) {
  px4_control_msgs::DroneStateMarker msg;

  // Header
  msg.header.stamp = time;

  // Position
  msg.pose.position.x = state.position(0);
  msg.pose.position.y = state.position(1);
  msg.pose.position.z = state.position(2);

  // Velocity
  msg.velocity.x = state.velocity(0);
  msg.velocity.y = state.velocity(1);
  msg.velocity.z = state.velocity(2);

  // Attitude
  msg.pose.orientation.w = state.attitude.w();
  msg.pose.orientation.x = state.attitude.x();
  msg.pose.orientation.y = state.attitude.y();
  msg.pose.orientation.z = state.attitude.z();

  // Disturbances
  msg.disturbances.x = state.disturbances(0);
  msg.disturbances.y = state.disturbances(1);
  msg.disturbances.z = state.disturbances(2);

  // Disturbances
  msg.random_walk_bias.x = state.random_walk_bias(0);
  msg.random_walk_bias.y = state.random_walk_bias(1);
  msg.random_walk_bias.z = state.random_walk_bias(2);

  // Marker
  msg.marker_found.data = marker_found;

  if (marker_found) {
    msg.marker_pose.position.x = state.marker_position(0);
    msg.marker_pose.position.y = state.marker_position(1);
    msg.marker_pose.position.z = state.marker_position(2);

    msg.marker_pose.orientation.w = state.marker_orientation.w();
    msg.marker_pose.orientation.x = state.marker_orientation.x();
    msg.marker_pose.orientation.y = state.marker_orientation.y();
    msg.marker_pose.orientation.z = state.marker_orientation.z();
  } else
    msg.marker_pose.orientation.w = 1.0;

  // Error covariance
  for (size_t i = 0; i < P_mat.rows(); i++) {
    msg.covariance.push_back(P_mat(i, i));
  }

  // Publish message
  state_pub.publish(msg);
}

}  // namespace px4_ctrl