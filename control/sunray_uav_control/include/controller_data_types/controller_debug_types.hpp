/* clang-format off */
/**
 * @file controller_debug_types.hpp
 * @brief brief description定义 Sunray 控制器核心层使用的调试数据类型。
 * @details 本文件旨在设计一个各控制器通用的debug数据类型，用于状态机向外发布或者记录控制器日志
 * 设计原则：
 * 1. 调试数据与控制输出解耦，避免控制器接口被 ROS 通信格式污染。
 * 2. 公共字段与控制器专有字段分离，降低后续扩展时的破坏性修改。
 * 3. 使用 valid_fields 区分“字段未提供”和“字段有效但数值恰好为 0”。
 * 4. 使用 flags 表示限幅、保护和特殊工况，避免大量零散 bool 字段。
-----------控制器在使用时-----------
struct Px4LocalControlResult {
  Px4LocalSetpoint command;
  Px4PositionControllerDebug debug;
};

Px4LocalControlResult calculateControl(
    const FlatTrajectoryPoint& des,
    const sunray_common::QuadStateEstimate& odom);

-----------状态机在使用时-----------
auto result = controller.calculateControl(des, odom);

publishSetpoint(result.command);
publishDebug(toRosMsg(result.debug));

 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 * 
 */
/* clang-format on */

#pragma once

#include <Eigen/Dense>
#include <cstdint>

namespace controller_data_types {

/// 所有控制器都可复用的通用调试信息。
struct ControllerDebugCommon {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_error = Eigen::Vector3d::Zero();
  double yaw_error = 0.0;
  double yaw_rate_error = 0.0;

  /// 字段有效位，由 controller_debug_utils.hpp 统一维护。
  uint32_t valid_fields = 0u;
};

/// 面向 `setpoint_raw/local` 输出路径的调试信息。
struct Px4LocalControlDebug {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControllerDebugCommon common;

  /// 误差反馈产生的修正加速度。
  Eigen::Vector3d feedback_acceleration = Eigen::Vector3d::Zero();
  /// 限幅前的最终加速度指令。
  Eigen::Vector3d commanded_acceleration = Eigen::Vector3d::Zero();
  /// 限幅后的实际采用加速度。
  Eigen::Vector3d limited_acceleration = Eigen::Vector3d::Zero();

  /// 最终给 local setpoint 的偏航与偏航角速度。
  double commanded_yaw = 0.0;
  double commanded_yaw_rate = 0.0;

  /// 字段有效位，由 controller_debug_utils.hpp 统一维护。
  uint32_t valid_fields = 0u;
};

/// 面向 `setpoint_raw/attitude` 输出路径的调试信息。
struct Px4AttitudeControlDebug {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControllerDebugCommon common;

  Eigen::Quaterniond reference_attitude = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond feedback_attitude = Eigen::Quaterniond::Identity();
  Eigen::Vector3d attitude_error = Eigen::Vector3d::Zero();

  Eigen::Vector3d reference_body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d feedback_body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector3d commanded_body_rate = Eigen::Vector3d::Zero();

  double reference_thrust = 0.0;
  double commanded_thrust = 0.0;

  /// 字段有效位，由 controller_debug_utils.hpp 统一维护。
  uint32_t valid_fields = 0u;
};

}  // namespace controller_data_types