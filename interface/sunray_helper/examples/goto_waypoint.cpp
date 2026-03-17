#include "sunray_helper/sunray_helper.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "goto_waypoint_example");
  ros::NodeHandle nh;
  SunrayHelper helper(nh);
  return helper.move_to(Eigen::Vector3d::Zero()) ? 0 : 1;
}
