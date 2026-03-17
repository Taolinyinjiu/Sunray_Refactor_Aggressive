/**
 * @file base_local_controller.cpp
 * @brief 本文主要希望实现基类控制器中的Px4LocalControlBase控制器
根据3.16的讨论，我们并不准备在这个base_local_controller中实现pid环节，反馈控制交给px4内部的控制环即可
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 *
 */
#include "controller/base_controller/base_controller.hpp"
#include "controller_data_types/controller_debug_utils.hpp"
#include "sunray_common/geometry_eigen_conversions.h"
// 令人难过的是，是用该环通常需要强调px4得到了外部里程计的数据，然而这里引入ros可能会导致耦合？
// 于是我们认为，is_ready函数只需要返回配置文件中fuse_odom_to_px4是否为true即可
bool Px4LocalControlBase::is_ready() const {
  return nh_.param("fuse_odom_to_px4", false);
}

// 这里简单地设计一个查询函数，用于寻找FlatTrajectoryPoint结构体中的有效数据
namespace {
bool hasField(const uint32_t valid_fields, const uint32_t field) {
  return (valid_fields & field) != 0u;
}
} // namespace

controller_data_types::Px4LocalControlResult
Px4LocalControlBase::calculateControl(
    const controller_data_types::FlatTrajectoryPoint &des,
    const sunray_common::QuadStateEstimate &odom) {
	// 构造输出变量
  controller_data_types::Px4LocalControlResult result;
	// 构造当前控制的坐标系为local系
  result.command.frame = controller_data_types::Px4LocalFrame::kLocalNed;
	// 默认先全部屏蔽掉，根据传入的有效位进行赋值
  result.command.mask = controller_data_types::Px4LocalSetpoint::kIgnorePx |
                        controller_data_types::Px4LocalSetpoint::kIgnorePy |
                        controller_data_types::Px4LocalSetpoint::kIgnorePz |
                        controller_data_types::Px4LocalSetpoint::kIgnoreVx |
                        controller_data_types::Px4LocalSetpoint::kIgnoreVy |
                        controller_data_types::Px4LocalSetpoint::kIgnoreVz |
                        controller_data_types::Px4LocalSetpoint::kIgnoreAfx |
                        controller_data_types::Px4LocalSetpoint::kIgnoreAfy |
                        controller_data_types::Px4LocalSetpoint::kIgnoreAfz |
                        controller_data_types::Px4LocalSetpoint::kIgnoreYaw |
                        controller_data_types::Px4LocalSetpoint::kIgnoreYawRate;
	// 如果位置帧有效，填入对应的期望位置参数
  if (hasField(des.valid_fields, controller_data_types::kPositionValid)) {
    result.command.position = des.position;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnorePx;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnorePy;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnorePz;

    result.debug.common.position_error = des.position - odom.position;
  }
	// 如果速度帧有效，填写对应的期望速度参数
  if (hasField(des.valid_fields, controller_data_types::kVelocityValid)) {
    result.command.velocity = des.velocity;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreVx;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreVy;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreVz;

    result.debug.common.velocity_error = des.velocity - odom.velocity;
  }
	// 如果加速度帧有效，填写对应的期望加速度参数
  if (hasField(des.valid_fields, controller_data_types::kAccelerationValid)) {
    result.command.accel_or_force = des.acceleration;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreAfx;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreAfy;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreAfz;

    result.debug.commanded_acceleration = des.acceleration;
    result.debug.limited_acceleration = des.acceleration;
  }
	// 如果yaw角帧有效，填写对应的期望yaw角参数
  if (hasField(des.valid_fields, controller_data_types::kYawValid)) {
    result.command.yaw = des.yaw;
    result.command.mask &= ~controller_data_types::Px4LocalSetpoint::kIgnoreYaw;

    result.debug.common.yaw_error = des.yaw - odom.getYaw();
    result.debug.commanded_yaw = des.yaw;
  }
	// 如果yaw角速度有效，填写对应的期望yaw角速度参数
  if (hasField(des.valid_fields, controller_data_types::kYawRateValid)) {
    result.command.yaw_rate = des.yaw_rate;
    result.command.mask &=
        ~controller_data_types::Px4LocalSetpoint::kIgnoreYawRate;

    result.debug.commanded_yaw_rate = des.yaw_rate;
  }
	// 返回输出变量
  return result;
};