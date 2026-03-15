#include "sunray_helper/sunray_robot_group.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_robot_group_square_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sunray_helper_fluent::Sunray_RobotGroup robot_group(nh);
  const char *tag = "RobotGroupSquareDemo";

  ROS_INFO("[%s] takeoff", tag);
  if (!robot_group.takeoff().wait_for_completed()) {
    return sunray_demo::fail(tag, "takeoff");
  }

  ros::Duration(1.0).sleep();
  const Eigen::Vector3d origin = robot_group.raw_helper().get_uav_position();
  const double initial_yaw = robot_group.raw_helper().get_uav_yaw_rad();

  const Eigen::Vector3d start = origin + Eigen::Vector3d(-1, 1, 0);
  ROS_INFO("[%s] move_to_square_start", tag);
  if (!robot_group.move_to(start, initial_yaw).wait_for_completed()) {
    return sunray_demo::fail(tag, "move_to_square_start");
  }

  std::vector<std::pair<Eigen::Vector3d, double>> square;
  for (const auto &point : sunray_demo::square_waypoints(origin)) {
    square.emplace_back(point, initial_yaw);
  }

  ROS_INFO("[%s] draw_square", tag);
  if (!robot_group.raw_helper().set_position_list_block(square)) {
    return sunray_demo::fail(tag, "draw_square");
  }
		
		ROS_INFO("[%s] return_home", tag);
		robot_group.return_home().wait_for_completed();
		ROS_INFO("[%s] return end", tag);
  return sunray_demo::done(tag, "done, hovering");
}
