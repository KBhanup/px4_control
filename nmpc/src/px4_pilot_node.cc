#include "nmpc/px4_pilot.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_pilot");
  ros::NodeHandle nh;
  px4_ctrl::PX4Pilot px4_pilot(nh, 10);

  ros::spin();

  px4_pilot.cmd_publisher_worker_t.join();

  return 0;
}
