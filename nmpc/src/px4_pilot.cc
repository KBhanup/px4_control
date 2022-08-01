#include "nmpc/px4_pilot.h"

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tf/transform_datatypes.h"

namespace px4_ctrl {

inline int loadSingleParameter(const ros::NodeHandle &nh,
                               const std::string &key,
                               const int &default_value) {
  int parameter;
  if (!nh.getParam(key, parameter)) {
    ROS_WARN("Could not retrieve %s. Setting to default", key.c_str());
    parameter = default_value;
  }
  return parameter;
}

inline double loadSingleParameter(const ros::NodeHandle &nh,
                                  const std::string &key,
                                  const double &default_value) {
  double parameter;
  if (!nh.getParam(key, parameter)) {
    ROS_WARN("Could not retrieve %s. Setting to default", key.c_str());
    parameter = default_value;
  }
  return parameter;
}

inline std::vector<double> loadVectorParameter(
    const ros::NodeHandle &nh, const std::string &key,
    const std::vector<double> &default_value) {
  std::vector<double> parameter;
  if (!nh.getParam(key, parameter)) {
    ROS_WARN("Could not retrieve %s. Setting to default", key.c_str());
    parameter = default_value;
  }
  return parameter;
}

PX4Pilot::PX4Pilot(ros::NodeHandle &nh, const double &rate) {
  // Initialize variables
  got_RC = false;
  drone_connected = false;
  allow_offboard = false;
  is_offboard = false;
  controller_enabled = false;
  has_drone_state = false;
  trajectory_loaded = false;

  current_reference_trajectory.clear();
  weights.clear();

  // Initialize mutexes
  status_mutex.reset(new std::mutex);
  rc_mutex.reset(new std::mutex);
  drone_state_mutex.reset(new std::mutex);

  // Load parameters
  loadParameters();

  // Initialize acados NMPC
  nmpc_controller = new AcadosNMPC();
  if (nmpc_controller->initializeController(model_params, input_lower_bound,
                                            input_upper_bound) &&
      nmpc_controller->setWeighingMatrix(weights)) {
    ROS_INFO("NMPC Initialized\n");
  } else {
    ROS_ERROR("Failed to initialize Acados NMPC. Exiting\n");
    exit(1);
  }

  // Initialize orientation backup controller
  o_pid = new PIDController(o_pid_k[0], o_pid_k[1], o_pid_k[2], 1);

  // Setup Subscribers
  mavros_status_sub =
      nh.subscribe("/mavros/state", 1, &PX4Pilot::mavrosStatusCallback, this);
  mavros_rc_sub =
      nh.subscribe("/mavros/rc/in", 1, &PX4Pilot::mavrosRCCallback, this);
  drone_state_sub =
      nh.subscribe("/drone_state", 1, &PX4Pilot::droneStateCallback, this);
  trajectory_sub =
      nh.subscribe("/drone_trajectory", 1, &PX4Pilot::trajectoryCallback, this);

  // Setup Publishers
  att_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>(
      "/mavros/setpoint_raw/attitude", 1);
  vel_control_pub = nh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 1);

  // Setup Service Servers
  enable_controller_server = nh.advertiseService(
      "/enable_controller", &PX4Pilot::enableControllerServCallback, this);

  // Setup Service Clients
  mavros_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_arm_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  // Start command publisher thread
  cmd_publisher_worker_t = std::thread(&PX4Pilot::commandPublisher, this, rate);
}

// Callbacks
void PX4Pilot::mavrosStatusCallback(const mavros_msgs::State::ConstPtr &msg) {
  std::lock_guard<std::mutex> status_guard(*(status_mutex));
  current_status = *msg;
}

void PX4Pilot::mavrosRCCallback(const mavros_msgs::RCIn::ConstPtr &msg) {
  if (!got_RC) {
    got_RC = true;
  }

  last_RC_time = msg->header.stamp;

  // Enable-Disable Controller
  if (msg->channels[controller_switch.channel] == controller_switch.on_value &&
      !controller_enabled && trajectory_loaded) {
    // Enable controller
    ROS_INFO("Enabling controller");
    controller_enabled = true;
  } else if (msg->channels[controller_switch.channel] ==
                 controller_switch.off_value &&
             controller_enabled) {
    // Disable controller
    ROS_INFO("Disabling controller");
    controller_enabled = false;
  }

  // Allow switching to Offboard
  if (msg->channels[offboard_switch.channel] == offboard_switch.on_value &&
      !allow_offboard) {
    // Allow sending cmds
    ROS_INFO("Allowing switch to OFFBOARD");
    allow_offboard = true;
  } else if (msg->channels[offboard_switch.channel] ==
                 offboard_switch.off_value &&
             allow_offboard) {
    // Block all offboard cmds
    ROS_INFO("Blocking all commands");
    allow_offboard = false;
  } else if (msg->channels[offboard_switch.channel] ==
                 offboard_switch.on_value &&
             allow_offboard && !is_offboard) {
    changeMode("OFFBOARD");
  }
}

void PX4Pilot::droneStateCallback(
    const px4_control_msgs::DroneStateMarker &msg) {
  std::lock_guard<std::mutex> state_guard(*(drone_state_mutex));
  drone_state.pos_x = msg.pose.position.x;
  drone_state.pos_y = msg.pose.position.y;
  drone_state.pos_z = msg.pose.position.z;
  drone_state.vel_x = msg.velocity.x;
  drone_state.vel_y = msg.velocity.y;
  drone_state.vel_z = msg.velocity.z;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.orientation, q);
  double y, p, r;
  tf::Matrix3x3(q).getEulerYPR(y, p, r);

  drone_state.q_yaw = y;
  drone_state.q_pitch = p;
  drone_state.q_roll = r;

  disturbances.clear();
  disturbances.push_back(msg.disturbances.x);
  disturbances.push_back(msg.disturbances.y);
  disturbances.push_back(msg.disturbances.z);

  last_state_time = msg.header.stamp;
  has_drone_state = !has_drone_state ? true : has_drone_state;
}

void PX4Pilot::trajectoryCallback(const px4_control_msgs::Trajectory &msg) {
  // Clear old trajectory
  current_reference_trajectory.clear();

  trajectory_setpoint setpoint;
  for (size_t i = 0; i < msg.trajectory.size(); i++) {
    setpoint.pos_x = msg.trajectory[i].position.x;
    setpoint.pos_y = msg.trajectory[i].position.y;
    setpoint.pos_z = msg.trajectory[i].position.z;
    setpoint.vel_x = msg.trajectory[i].velocity.x;
    setpoint.vel_y = msg.trajectory[i].velocity.y;
    setpoint.vel_z = msg.trajectory[i].velocity.z;
    setpoint.q_roll = msg.trajectory[i].orientation.x;
    setpoint.q_pitch = msg.trajectory[i].orientation.y;
    setpoint.q_yaw = msg.trajectory[i].orientation.z;

    current_reference_trajectory.push_back(setpoint);
  }
  // Stop controller while loading the new trajectory
  if (controller_enabled) {
    controller_enabled = false;
    nmpc_controller->setTrajectory(current_reference_trajectory);
    controller_enabled = true;
  } else
    nmpc_controller->setTrajectory(current_reference_trajectory);

  ROS_INFO("Trajectory loaded");
  trajectory_loaded = true;
}

bool PX4Pilot::enableControllerServCallback(std_srvs::SetBool::Request &req,
                                            std_srvs::SetBool::Response &res) {
  if (trajectory_loaded & req.data) {
    ROS_WARN("Controller enabled");
    controller_enabled = true;
    res.success = true;
    return true;
  } else if (!trajectory_loaded & req.data) {
    ROS_WARN("No setpoint or trajectory is loaded. Load one and try again");
    res.success = false;
    return false;
  } else if (!req.data) {
    ROS_WARN("Controller disabled");
    controller_enabled = false;
    res.success = true;
    return true;
  }
  ROS_ERROR("Unexpected behavior");
  res.success = false;
  return false;
}

void PX4Pilot::loadParameters() {
  // Create private nodehandle to load parameters
  ros::NodeHandle nh_pvt("~");

  // Controller parameters
  model_params.t_pitch = loadSingleParameter(nh_pvt, "t_pitch", 1.0);
  model_params.k_pitch = loadSingleParameter(nh_pvt, "k_pitch", 1.0);
  model_params.t_roll = loadSingleParameter(nh_pvt, "t_roll", 1.0);
  model_params.k_roll = loadSingleParameter(nh_pvt, "k_roll", 1.0);
  model_params.k_thrust = loadSingleParameter(nh_pvt, "k_thrust", 1.0);
  model_params.gravity = loadSingleParameter(nh_pvt, "gravity", -9.8066);

  std::vector<double> vector_parameter{0.0, 0.0, 0.0};
  std::vector<double> damping_coef =
      loadVectorParameter(nh_pvt, "damping_coef", vector_parameter);
  model_params.damp_x = damping_coef[0];
  model_params.damp_y = damping_coef[1];
  model_params.damp_z = damping_coef[2];

  // Controller weights
  std::vector<double> pos_w =
      loadVectorParameter(nh_pvt, "pos_w", vector_parameter);
  std::vector<double> vel_w =
      loadVectorParameter(nh_pvt, "vel_w", vector_parameter);
  std::vector<double> att_w =
      loadVectorParameter(nh_pvt, "att_w", vector_parameter);
  double yaw_rate_cmd_w = loadSingleParameter(nh_pvt, "yaw_rate_cmd_w", 100.0);
  double pitch_cmd_w = loadSingleParameter(nh_pvt, "pitch_cmd_w", 100.0);
  double roll_cmd_w = loadSingleParameter(nh_pvt, "roll_cmd_w", 100.0);
  double thrust_cmd_w = loadSingleParameter(nh_pvt, "thrust_cmd_w", 100.0);

  std::vector<double> default_gains{0.0, 0.0};
  std::vector<double> gains;
  gains = loadVectorParameter(nh_pvt, "x_gain", default_gains);
  x_kp = gains[0];
  x_kv = gains[1];
  gains = loadVectorParameter(nh_pvt, "y_gain", default_gains);
  y_kp = gains[0];
  y_kv = gains[1];
  gains = loadVectorParameter(nh_pvt, "z_gain", default_gains);
  z_kp = gains[0];
  z_kv = gains[1];
  o_pid_k = loadVectorParameter(nh_pvt, "o_pid", vector_parameter);

  // Controller input constraints
  input_lower_bound = loadVectorParameter(nh_pvt, "lbu", default_gains);
  input_upper_bound = loadVectorParameter(nh_pvt, "ubu", default_gains);

  // Cost function weights
  weights.push_back(pos_w[0]);
  weights.push_back(pos_w[1]);
  weights.push_back(pos_w[2]);
  weights.push_back(vel_w[0]);
  weights.push_back(vel_w[1]);
  weights.push_back(vel_w[2]);
  weights.push_back(att_w[0]);
  weights.push_back(att_w[1]);
  weights.push_back(att_w[2]);
  weights.push_back(yaw_rate_cmd_w);
  weights.push_back(pitch_cmd_w);
  weights.push_back(roll_cmd_w);
  weights.push_back(thrust_cmd_w);

  // RC
  // Controller switch
  controller_switch.channel = loadSingleParameter(nh_pvt, "auto_channel", 4);
  controller_switch.on_value = loadSingleParameter(nh_pvt, "auto_on", 982);
  controller_switch.off_value = loadSingleParameter(nh_pvt, "auto_off", 2006);

  // Offboard switch
  offboard_switch.channel = loadSingleParameter(nh_pvt, "offboard_channel", 5);
  offboard_switch.on_value = loadSingleParameter(nh_pvt, "offboard_on", 2006);
  offboard_switch.off_value = loadSingleParameter(nh_pvt, "offboard_off", 1494);
}

void PX4Pilot::changeMode(const std::string &mode) {
  ros::Rate rate(5);
  mavros_msgs::SetMode status_mode;
  status_mode.request.custom_mode = mode;

  for (int i = 20; ros::ok() & i > 0; --i) {
    if (mavros_mode_client.call(status_mode) &&
        status_mode.response.mode_sent) {
      ROS_INFO("Flight mode changed to %s", mode.c_str());
      break;
    }
    rate.sleep();
  }
}

void PX4Pilot::commandPublisher(const double &pub_rate) {
  ros::Rate rate(pub_rate);

  // Make sure vehicle is connected
  ROS_INFO("Connecting to vehicle");
  while (ros::ok()) {
    std::lock_guard<std::mutex> status_guard(*(status_mutex));
    drone_connected = current_status.connected;
    if (drone_connected) break;
  }
  ROS_INFO("Vehicle is connected");

  // Setup command msgs
  // Attitude
  mavros_msgs::AttitudeTarget att_cmd;
  att_cmd.type_mask = att_cmd.IGNORE_ROLL_RATE | att_cmd.IGNORE_PITCH_RATE;

  // Velocity
  mavros_msgs::PositionTarget vel_cmd;
  vel_cmd.coordinate_frame = vel_cmd.FRAME_BODY_NED;
  vel_cmd.type_mask = vel_cmd.IGNORE_PX | vel_cmd.IGNORE_PY |
                      vel_cmd.IGNORE_PZ | vel_cmd.IGNORE_AFX |
                      vel_cmd.IGNORE_AFY | vel_cmd.IGNORE_AFZ |
                      vel_cmd.IGNORE_YAW;

  while (ros::ok()) {
    {  // Lock status mutex
      std::lock_guard<std::mutex> status_guard(*(status_mutex));
      is_offboard = current_status.mode == "OFFBOARD";
      drone_connected = current_status.connected;
    }

    // Chek if something went wrong while in offboard mode
    if (is_offboard) {
      if (got_RC && (ros::Time::now() - last_RC_time).toSec() > 0.5) {
        got_RC = false;
      }

      // Drone state lost
      if (((ros::Time::now() - last_state_time).toSec() > 0.5)) {
        ROS_WARN(
            "Have not received the drone state for a while!!! Switching to "
            "Position Control");
        changeMode("POSCTL");
        continue;
      }

      // Connection to drone lost
      if (!drone_connected) {
        ROS_WARN("Lost Connection to Vehicle!!! Switching to Position Control");
        changeMode("POSCTL");
        continue;
      }

      // RC lost
      if (!got_RC) {
        ROS_WARN("Lost RC!!! Switching to Position Control");
        changeMode("POSCTL");
        continue;
      }
    }

    if (allow_offboard) {
      if (controller_enabled && has_drone_state && is_offboard) {
        // Update current state
        double current_yaw;
        double error_time = ros::Time::now().toSec();
        {  // Lock state mutex
          std::lock_guard<std::mutex> state_guard(*(drone_state_mutex));
          current_yaw = drone_state.q_yaw;
          nmpc_controller->setCurrentState(drone_state, disturbances);
        }

        // Send controller commands
        std::vector<double> ctrl;
        if (nmpc_controller->getCommands(ctrl)) {
          tf2::Quaternion q;
          q.setRPY(ctrl[2], ctrl[1], current_yaw);
          q.normalize();

          att_cmd.orientation.x = q[0];
          att_cmd.orientation.y = q[1];
          att_cmd.orientation.z = q[2];
          att_cmd.orientation.w = q[3];
          att_cmd.body_rate.z = ctrl[0];
          att_cmd.thrust = ctrl[3] < 0.1 ? 0.1 : ctrl[3];

          att_cmd.header.stamp = ros::Time::now();
          att_control_pub.publish(att_cmd);
        } else {
          ROS_ERROR("NMPC failed. Using backup controller");
          trajectory_setpoint current_setpoint =
              nmpc_controller->getCurrentSetpoint();

          double syaw = sin(current_yaw);
          double cyaw = cos(current_yaw);
          double d_px = current_setpoint.pos_x - drone_state.pos_x;
          double d_vx = current_setpoint.vel_x - drone_state.vel_x;
          double d_py = current_setpoint.pos_y - drone_state.pos_y;
          double d_vy = current_setpoint.vel_y - drone_state.vel_y;
          double d_pz = current_setpoint.pos_z - drone_state.pos_z;
          double d_vz = current_setpoint.vel_z - drone_state.vel_z;

          vel_cmd.velocity.x = x_kp * (cyaw * d_px - syaw * d_py) +
                               x_kv * (cyaw * d_vx - syaw * d_vy);
          vel_cmd.velocity.y = y_kp * (syaw * d_px + cyaw * d_py) +
                               y_kv * (syaw * d_vx + cyaw * d_vy);
          vel_cmd.velocity.z = z_kp * d_pz + z_kv * d_vz;
          vel_cmd.yaw_rate = o_pid->getControl(
              current_setpoint.q_yaw - current_yaw, error_time);

          vel_cmd.header.stamp = ros::Time::now();
          vel_control_pub.publish(vel_cmd);
        }
      } else {
        // Send zero velocity commands so that it can be switched to Offboard
        vel_cmd.velocity.x = 0.0;
        vel_cmd.velocity.y = 0.0;
        vel_cmd.velocity.z = 0.0;
        vel_cmd.yaw_rate = 0.0;
        vel_cmd.header.stamp = ros::Time::now();
        vel_control_pub.publish(vel_cmd);
      }
    }
    rate.sleep();
  }
}

}  // namespace px4_ctrl
