#include "sunray_helper/sunray_robot_group.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_robot_group_pentagram_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  sunray_helper_fluent::Sunray_RobotGroup robot_group(nh);
  const char *tag = "RobotGroupPentagramDemo";

  if (!robot_group.takeoff().wait_for_completed()) {
    return sunray_demo::fail(tag, "takeoff");
  }

  const Eigen::Vector3d origin = robot_group.raw_helper().get_uav_position();
  const Eigen::Vector3d start = sunray_demo::offset_xy(origin, -1.0, 1.0);
  if (!robot_group.move_to(start).wait_for_completed()) {
    return sunray_demo::fail(tag, "move_to_star_start");
  }

  std::vector<std::pair<Eigen::Vector3d, double>> pentagram;
  for (const auto &point : sunray_demo::pentagram_waypoints(start)) {
    pentagram.push_back(std::make_pair(point, sunray_demo::kNominalSpeedMps));
  }

  if (!robot_group.raw_helper().set_position_velocity_list_async(pentagram) ||
      !sunray_demo::wait_for_hover_and_position(
          robot_group.raw_helper(), start,
          sunray_demo::path_timeout_s(start, pentagram))) {
    return sunray_demo::fail(tag, "draw_pentagram");
  }

  return sunray_demo::done(tag, "done, hovering");
}
