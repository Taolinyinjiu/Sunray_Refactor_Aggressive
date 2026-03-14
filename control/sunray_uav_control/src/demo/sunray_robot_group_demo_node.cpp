#include "sunray_helper/sunray_robot_group.h"

#include <ros/ros.h>

#include <Eigen/Dense>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_robot_group_demo_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  sunray_helper_fluent::Sunray_RobotGroup robot_group(nh);

  const auto log_current_position = [&robot_group](const char *stage) {
    const Eigen::Vector3d position = robot_group.raw_helper().get_uav_position();
    ROS_WARN("[SunrayRobotGroupDemo] stop after %s, current position=(%.3f, %.3f, %.3f)",
             stage, position.x(), position.y(), position.z());
  };

  ROS_INFO("[SunrayRobotGroupDemo] takeoff");
  if (!robot_group.takeoff().wait_for_completed()) {
    ROS_WARN("[SunrayRobotGroupDemo] takeoff did not finish cleanly");
    log_current_position("takeoff");
    ros::shutdown();
    return 1;
  }

  const Eigen::Vector3d start_pos = robot_group.raw_helper().get_uav_position();
  const Eigen::Vector3d forward_pos =
      start_pos + Eigen::Vector3d(1.0, 0.0, 0.0);
  ROS_INFO("[SunrayRobotGroupDemo] move forward 1m");
  if (!robot_group.move_to(forward_pos).wait_for_completed()) {
    ROS_WARN("[SunrayRobotGroupDemo] forward move did not finish cleanly");
    log_current_position("move forward");
    ros::shutdown();
    return 1;
  }

  const Eigen::Vector3d right_pos =
      forward_pos + Eigen::Vector3d(0.0, -1.0, 0.0);
  ROS_INFO("[SunrayRobotGroupDemo] move right 1m");
  if (!robot_group.move_to(right_pos).wait_for_completed()) {
    ROS_WARN("[SunrayRobotGroupDemo] right move did not finish cleanly");
    log_current_position("move right");
    ros::shutdown();
    return 1;
  }

  ROS_INFO("[SunrayRobotGroupDemo] land");
  if (!robot_group.return_home().wait_for_completed()) {
    ROS_WARN("[SunrayRobotGroupDemo] land did not finish cleanly");
    log_current_position("land");
    ros::shutdown();
    return 1;
  }

  ROS_INFO("[SunrayRobotGroupDemo] done");
  ros::shutdown();
  return 0;
}
