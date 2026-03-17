/**
 * @file sunray_attitude_controller.hpp
 * @brief 本文件作为Px4AttitudeControlBase
派生指南，旨在实现一个从Px4AttitudeControlBase类排生的控制器
1. 将输出量由attitude+thrust 重写为 bodyrate+thrust
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 *
 */

#pragma once

#include "controller/base_controller/base_controller.hpp"

class Sunray_BodyRate_Controller : public Px4AttitudeControlBase {
public:
  explicit Sunray_BodyRate_Controller(ros::NodeHandle &nh)
      : Px4AttitudeControlBase(nh) {}

  controller_data_types::Px4AttitudeControlResult calculateControl(
      const controller_data_types::FlatTrajectoryPoint &des,
      const sunray_common::QuadStateEstimate &odom,
      const sensor_msgs::Imu &imu) override {
    (void)des;
    (void)odom;
    (void)imu;
    return {};
  }
};
