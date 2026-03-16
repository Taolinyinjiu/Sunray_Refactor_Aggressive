#include <ros/ros.h>

#include <Eigen/Geometry>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#include "sunray_common/geometry_eigen_conversions.h"
#include "sunray_common/quad_state_estimate.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_common_ros_compile_test_node",
            ros::init_options::AnonymousName | ros::init_options::NoRosout);
  ros::NodeHandle nh("~");

  geometry_msgs::Vector3 velocity_msg;
  velocity_msg.x = 1.0;
  velocity_msg.y = 2.0;
  velocity_msg.z = 3.0;
  const Eigen::Vector3d velocity =
      sunray_common::geometryToEigen(velocity_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_msg.twist.twist.linear = velocity_msg;

  sunray_common::QuadStateEstimate state(odom_msg);
  state.transformVelocityToWorldFrame();

  (void)nh;
  (void)velocity;
  (void)state;

  ros::shutdown();
  return 0;
}
