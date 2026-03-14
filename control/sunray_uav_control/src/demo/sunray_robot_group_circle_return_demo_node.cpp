#include "sunray_helper/sunray_robot_group.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_robot_group_circle_return_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sunray_helper_fluent::Sunray_RobotGroup robot_group(nh);
  const char *tag = "RobotGroupCircleReturnDemo";

  if (!robot_group.takeoff().wait_for_completed()) {
    return sunray_demo::fail(tag, "takeoff");
  }

  const Eigen::Vector3d origin = robot_group.raw_helper().get_uav_position();
  ros::Duration(1.0).sleep();

  const Eigen::Vector3d start = sunray_demo::offset_xy(origin, -1.0, 0.0);
  if (!robot_group.move_to(start).wait_for_completed()) {
    return sunray_demo::fail(tag, "move_to_circle_start");
  }

  if (!sunray_demo::run_trajectory(
          robot_group, sunray_demo::make_trajectory(
                           start, sunray_demo::circle_waypoints(origin)))) {
    return sunray_demo::fail(tag, "draw_circle");
  }

  if (!robot_group.return_home().wait_for_completed()) {
    return sunray_demo::fail(tag, "return_and_land");
  }

  return sunray_demo::done(tag, "done");
}
