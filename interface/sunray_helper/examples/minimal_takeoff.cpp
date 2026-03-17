#include "sunray_helper/sunray_helper.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "minimal_takeoff_example");
  ros::NodeHandle nh;
  SunrayHelper helper(nh);
  return helper.takeoff() ? 0 : 1;
}
