/**
 * @file quad_state_estimate.h
 * @brief 本文件旨在提供一个辅助结构体，快速的对nav_msgs::Odometry类型的ros消息进行解析并转换为Eigen类型数据
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 * 
 */

#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace sunray_common {

struct QuadStateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadStateEstimate();
  QuadStateEstimate(const nav_msgs::Odometry &state_estimate_msg);
	~QuadStateEstimate() = default;

  nav_msgs::Odometry toRosMessage() const;
	// 将速度从机体系转换为惯性系
  void transform_VelocityToWorldFrame();
  double getYaw() const;
  bool isValid() const;

  ros::Time timestamp;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyrates; // Body rates are represented in body coordinates
};

} // namespace sunray_common
