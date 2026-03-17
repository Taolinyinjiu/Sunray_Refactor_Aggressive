/**
 * @file controller_debug_fields.hpp
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @brief 定义 Sunray 控制器调试数据的字段有效位。
 *
 * @version 0.1
 * @date 2026-03-17
 */

#pragma once

#include <cstdint>

namespace controller_data_types {

/// ControllerDebugCommon 各字段的有效位定义。
enum ControllerDebugCommonField : uint32_t {
  kPositionErrorValid = 1u << 0,
  kVelocityErrorValid = 1u << 1,
  kYawErrorValid = 1u << 2,
  kYawRateErrorValid = 1u << 3
};

/// Px4LocalControlDebug 各字段的有效位定义。
enum Px4LocalControlDebugField : uint32_t {
  kFeedbackAccelerationValid = 1u << 0,
  kCommandedAccelerationValid = 1u << 1,
  kLimitedAccelerationValid = 1u << 2,
  kCommandedYawValid = 1u << 3,
  kCommandedYawRateValid = 1u << 4
};

/// Px4AttitudeControlDebug 各字段的有效位定义。
enum Px4AttitudeControlDebugField : uint32_t {
  kReferenceAttitudeValid = 1u << 0,
  kFeedbackAttitudeValid = 1u << 1,
  kAttitudeErrorValid = 1u << 2,
  kReferenceBodyRateValid = 1u << 3,
  kFeedbackBodyRateValid = 1u << 4,
  kCommandedBodyRateValid = 1u << 5,
  kReferenceThrustValid = 1u << 6,
  kCommandedThrustValid = 1u << 7
};

}  // namespace controller_data_types