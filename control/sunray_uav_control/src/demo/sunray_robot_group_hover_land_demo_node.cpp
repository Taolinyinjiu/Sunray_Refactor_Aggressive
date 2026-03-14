#include "sunray_helper/sunray_robot_group.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_robot_group_hover_land_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sunray_helper_fluent::Sunray_RobotGroup robot_group(nh);
  const char *tag = "RobotGroupHoverLandDemo";

  if (!robot_group.takeoff(0.0, 0.0).wait_for_completed()) {
    return sunray_demo::fail(tag, "takeoff");
  }

  ros::Duration(5.0).sleep();

  if (!robot_group.land(0, 0.0).wait_for_completed()) {
    return sunray_demo::fail(tag, "land");
  }

  return sunray_demo::done(tag, "done");
}
