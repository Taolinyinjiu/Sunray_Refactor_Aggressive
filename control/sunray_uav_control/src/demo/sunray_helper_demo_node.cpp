#include "sunray_helper/sunray_helper.h"

#include <ros/ros.h>

#include <Eigen/Dense>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_helper_demo_node");
  ros::NodeHandle nh;

  int land_type = 0;
  nh.param("land_type", land_type, land_type);

  Sunray_Helper helper(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("[SunrayHelperDemo] takeoff");
  if (!helper.takeoff_block()) {
    ROS_WARN("[SunrayHelperDemo] takeoff request rejected or service unavailable");
  }

  Eigen::Vector3d start_pos = helper.get_uav_position();
  Eigen::Vector3d forward_pos = start_pos + Eigen::Vector3d(1.0, 0.0, 0.0);
  ROS_INFO("[SunrayHelperDemo] move forward 1m");
  if (!helper.set_position_block(forward_pos)) {
    ROS_WARN("[SunrayHelperDemo] forward move did not finish cleanly");
  }

  Eigen::Vector3d right_pos = forward_pos + Eigen::Vector3d(0.0, -1.0, 0.0);
  ROS_INFO("[SunrayHelperDemo] move right 1m");
  if (!helper.set_position_block(right_pos)) {
    ROS_WARN("[SunrayHelperDemo] right move did not finish cleanly");
  }

  ROS_INFO("[SunrayHelperDemo] land");
  if (!helper.return_block()) {
    ROS_WARN("[SunrayHelperDemo] land request rejected or service unavailable");
  }

  ROS_INFO("[SunrayHelperDemo] done");
  ros::shutdown();
  return 0;
}
