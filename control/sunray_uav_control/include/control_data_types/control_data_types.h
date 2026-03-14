/**
 * @file control_data_types.h
 * @brief Sunray 控制链路通用数据类型定义。
 *
 * @details
 * 本文件集中定义控制状态枚举、控制器输入轨迹点和控制器标准输出，
 * 用于状态机、控制器和执行层之间的数据交换。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/duration.h"
#include <Eigen/Dense>
#include <cstdint>
#include <ros/time.h>
#include <string>
#include <vector>

namespace uav_control {

/**
 * @brief 任务状态机状态定义（面向 FSM 的顶层状态）。
 */
enum class ControlState {
  UNDEFINED,
  OFF,
  TAKEOFF,
  LAND,
  EMERGENCY_LAND,
  RETURN,
  HOVER,
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  ATTITUDE_CONTROL,
  COMPLEX_CONTROL,
  TRAJECTORY_CONTROL
};

/**
 * @brief 控制器内部状态定义（面向控制律阶段切换）。
 *
 * @details
 * 与 `ControlState` 区分：该枚举聚焦于控制器本体行为，
 * 用于控制器内部在起飞/悬停/移动/降落等阶段选择不同控制输出策略。
 */
enum class ControllerState {
  UNDEFINED,
  OFF,
  TAKEOFF,
  LAND,
  EMERGENCY_LAND,
  HOVER,
  MOVE
};

/**
 * @brief 控制输出掩码位定义。
 */
enum class ControllerOutputMask : uint32_t {
  UNDEFINED = 0U,
  POSITION = 1U << 0, ///< position 字段有效
  VELOCITY = 1U << 1, ///< velocity 字段有效
  ACCELERATION = 1U << 2,
  FORCE = 1U << 3,
  YAW = 1U << 4,
  YAW_RATE = 1U << 5,
  ATTITUDE = 1U << 6, ///< attitude 字段有效
  BODY_RATE = 1U << 7,
  THRUST = 1U << 8 ///< thrust 字段有效
};

// ControllerOutput output;
// output.channel_enable(uav_control::ControllerOutputMase);

/**
 * @brief 控制器标准输出。
 * @note 各字段是否有效由 output_mask 指示，FSM/执行器应按掩码消费数据。
 */
struct ControllerOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControllerOutput() = default;
  ~ControllerOutput() = default;

  /**
   * @brief 使能某个输出子项。
   * @param item 需要使能的掩码位。
   */
  void channel_enable(ControllerOutputMask item);

  /**
   * @brief 关闭某个输出子项。
   * @param item 需要关闭的掩码位。
   */
  void channel_disable(ControllerOutputMask item);

  /**
   * @brief 判断某个输出子项是否已使能。
   * @param item 掩码位。
   * @return true 已使能；false 未使能。
   */
  bool is_channel_enabled(ControllerOutputMask item) const;

  /**
   * @brief 清除所有输出字段并重置 `output_mask`。
   */
  void clear_all(void);

  // PositionTarget 对应字段
  Eigen::Vector3d position = Eigen::Vector3d::Zero(); ///< POSITION
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); ///< VELOCITY
  Eigen::Vector3d acceleration_or_force =
      Eigen::Vector3d::Zero(); ///< ACCELERATION / FORCE
  double yaw = 0.0;            ///< YAW
  double yaw_rate = 0.0;       ///< YAW_RATE

  // AttitudeTarget 对应字段
  Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity(); ///< ATTITUDE
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();          ///< BODY_RATE
  double thrust = 0.0;                                          ///< THRUST

  uint32_t output_mask = static_cast<uint32_t>(ControllerOutputMask::UNDEFINED);
};

// 是否应当在这里表示出控制器的期望输出，或者说控制器的期望输出是什么样的？

}; // namespace uav_control
