#include "sunray_helper/sunray_helper.h"
#include "sunray_demo_utils.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_helper_circle_return_demo_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Sunray_Helper helper(nh);
  const char *tag = "HelperCircleReturnDemo";

  if (!helper.takeoff_block()) {
    return sunray_demo::fail(tag, "takeoff");
  }

  const Eigen::Vector3d origin = helper.get_uav_position();
  ros::Duration(1.0).sleep();

  const Eigen::Vector3d start = sunray_demo::offset_xy(origin, -1.0, 0.0);
  if (!helper.set_position_block(start)) {
    return sunray_demo::fail(tag, "move_to_circle_start");
  }

  if (!sunray_demo::run_trajectory(
          helper, sunray_demo::make_trajectory(
                      start, sunray_demo::circle_waypoints(origin)))) {
    return sunray_demo::fail(tag, "draw_circle");
  }

  if (!helper.return_block()) {
    return sunray_demo::fail(tag, "return_and_land");
  }

  return sunray_demo::done(tag, "done");
}
