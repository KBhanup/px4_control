#pragma once

#include <mutex>
#include <thread>

// Acados NMPC
#include "nmpc/acados_nmpc.h"
#include "nmpc/pid.h"

// ROS
#include "ros/ros.h"

// ROS messages
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <std_srvs/SetBool.h>

#include "px4_control_msgs/DroneStateMarker.h"
#include "px4_control_msgs/Setpoint.h"
#include "px4_control_msgs/Trajectory.h"

namespace px4_ctrl {

struct rc_switch {
  uint8_t channel;
  uint16_t on_value;
  uint16_t off_value;
};

class PX4Pilot {
 public:
  PX4Pilot(ros::NodeHandle &nh, const double &rate);
  ~PX4Pilot() { delete nmpc_controller; }

  std::thread cmd_publisher_worker_t;

 private:
  // ROS Subscribers
  ros::Subscriber mavros_status_sub;
  ros::Subscriber mavros_rc_sub;
  ros::Subscriber drone_state_sub;
  ros::Subscriber trajectory_sub;

  // ROS Publishers
  ros::Publisher att_control_pub;
  ros::Publisher vel_control_pub;

  // ROS Service Servers
  ros::ServiceServer enable_controller_server;

  // ROS Service Clients
  ros::ServiceClient mavros_mode_client;
  ros::ServiceClient mavros_arm_client;

  // Subscriber Callbacks
  void mavrosStatusCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavrosRCCallback(const mavros_msgs::RCIn::ConstPtr &msg);
  void droneStateCallback(const px4_control_msgs::DroneStateMarker &msg);
  void trajectoryCallback(const px4_control_msgs::Trajectory &msg);

  // Service Callbacks
  bool enableControllerServCallback(std_srvs::SetBool::Request &req,
                                    std_srvs::SetBool::Response &res);

  /**
   * @brief Loads the node parameters
   */
  void loadParameters();

  /**
   * @brief Sends a mode change request
   * @param mode Flight mode:{ALTCTL, POSCTL, OFFBOARD, AUTO.TAKEOFF, AUTO.LAND}
   */
  void changeMode(const std::string &mode);

  /**
   * @brief If tracking is enabled, it publishes the UAS commands. Should run as
   * long as the node is alive
   * @param pub_rate Publishing rate
   */
  void commandPublisher(const double &pub_rate);

  // PX4 Status
  mavros_msgs::State current_status;
  bool drone_connected;
  bool is_offboard;

  // PX4 RC
  bool got_RC;
  ros::Time last_RC_time;
  rc_switch controller_switch;
  rc_switch offboard_switch;

  // PX4 Controller
  AcadosNMPC *nmpc_controller;
  bool has_drone_state;
  bool controller_enabled;
  bool allow_offboard;
  bool trajectory_loaded;
  ros::Time last_state_time;

  // Backup controller gains
  double x_kp, x_kv;
  double y_kp, y_kv;
  double z_kp, z_kv;
  PIDController *o_pid;
  std::vector<double> o_pid_k;

  model_parameters model_params;
  trajectory_setpoint drone_state;
  std::vector<double> disturbances, weights;
  std::vector<double> input_lower_bound, input_upper_bound;
  std::vector<trajectory_setpoint> current_reference_trajectory;

  // Mutexes
  std::unique_ptr<std::mutex> status_mutex;
  std::unique_ptr<std::mutex> rc_mutex;
  std::unique_ptr<std::mutex> drone_state_mutex;
};
}  // namespace px4_ctrl