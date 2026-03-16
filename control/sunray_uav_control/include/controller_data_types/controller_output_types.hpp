/**
 * @file controller_output_types.hpp
 * @author Taolinyinjiu @YunDrone Tech (taolinyinjiu@foxgmail.com)
 * @brief Sunray 控制器输出边界定义。
 *
 * 控制器对状态机只暴露两类最终输出：
 * - `setpoint_raw/local`
 * - `setpoint_raw/attitude`
 *
 * 上层接口可以是位置、速度、轨迹或自定义掩码，但最终都应归一到这两种输出之一。
 *
 * @version 0.1
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 */

#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace controller_data_types {

/// `setpoint_raw/local` 使用的坐标系，与 MAVROS PositionTarget 保持一致。
enum class LocalSetpointFrame : uint8_t {
  kLocalNed = 1,
  kLocalOffsetNed = 7,
  kBodyNed = 8,
  kBodyOffsetNed = 9
};

/// 对应 `setpoint_raw/local` 的控制器输出。
struct LocalSetpointOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// 与 MAVROS PositionTarget::type_mask 对齐。
  enum TypeMask : uint16_t {
    kIgnorePx = 1u,
    kIgnorePy = 2u,
    kIgnorePz = 4u,
    kIgnoreVx = 8u,
    kIgnoreVy = 16u,
    kIgnoreVz = 32u,
    kIgnoreAfx = 64u,
    kIgnoreAfy = 128u,
    kIgnoreAfz = 256u,
    kForceSetpoint = 512u,
    kIgnoreYaw = 1024u,
    kIgnoreYawRate = 2048u
  };

  /// setpoint 中位置/速度/加速度字段的解释坐标系。
  LocalSetpointFrame coordinate_frame = LocalSetpointFrame::kLocalNed;
  /// 指定哪些字段参与控制，哪些字段被忽略。
  uint16_t type_mask = 0;

  /// 目标位置。
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// 目标速度。
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  /// 目标加速度，或在 `kForceSetpoint` 置位时表示目标力。
  Eigen::Vector3d acceleration_or_force = Eigen::Vector3d::Zero();

  /// 目标偏航角。
  double yaw = 0.0;
  /// 目标偏航角速度。
  double yaw_rate = 0.0;
};

/// 对应 `setpoint_raw/attitude` 的控制器输出。
struct AttitudeSetpointOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// 与 MAVROS AttitudeTarget::type_mask 对齐。
  enum TypeMask : uint8_t {
    kIgnoreRollRate = 1u,
    kIgnorePitchRate = 2u,
    kIgnoreYawRate = 4u,
    kIgnoreThrust = 64u,
    kIgnoreAttitude = 128u
  };

  /// 指定姿态、角速度、推力中哪些字段参与控制。
  uint8_t type_mask = 0;

  /// 目标姿态。
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
  /// 机体系目标角速度。
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
  /// 归一化总推力。
  double thrust = 0.0;
};

}  // namespace controller_data_types
