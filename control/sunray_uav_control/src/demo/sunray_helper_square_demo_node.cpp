#include "sunray_helper/sunray_helper.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_helper_square_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Sunray_Helper helper(nh);
  const char *tag = "HelperSquareDemo";

  if (!helper.takeoff_block()) {
    return sunray_demo::fail(tag, "takeoff");
  }

  const Eigen::Vector3d origin = helper.get_uav_position();
  ros::Duration(1.0).sleep();

  const Eigen::Vector3d start = sunray_demo::offset_xy(origin, -1.0, 1.0);
  if (!helper.set_position_block(start, 0.0)) {
    return sunray_demo::fail(tag, "move_to_square_start");
  }

  const std::vector<Eigen::Vector3d> square_waypoints =
      sunray_demo::square_waypoints(origin);
  std::vector<std::pair<Eigen::Vector3d, double>> square;
  for (const auto &point : square_waypoints) {
    square.push_back(std::make_pair(point, 0.0));
  }

  if (!helper.set_position_list_async(square) ||
      !sunray_demo::wait_for_hover_and_position(
          helper, start, sunray_demo::path_timeout_s(start, square_waypoints))) {
    return sunray_demo::fail(tag, "draw_square");
  }

  return sunray_demo::done(tag, "done, hovering");
}
