#include "state_estimation/state_observer.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_observer_node");
  ros::NodeHandle nh;
  px4_ctrl::StateObserver observer(nh);

  ros::spin();

  return 0;
}