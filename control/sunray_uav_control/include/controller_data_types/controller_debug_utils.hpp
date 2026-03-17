/**
 * @file controller_debug_utils.hpp
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @brief 为控制器调试数据提供语义化的 set/has/clear 操作。
 *
 * @version 0.1
 * @date 2026-03-17
 */

#pragma once

#include "controller_data_types/controller_debug_fields.hpp"
#include "controller_data_types/controller_debug_types.hpp"

namespace controller_data_types {
namespace debug_utils {
namespace detail {

inline bool hasField(const uint32_t valid_fields, const uint32_t field) {
  return (valid_fields & field) != 0u;
}

inline void markField(uint32_t &valid_fields, const uint32_t field) {
  valid_fields |= field;
}

inline void clearField(uint32_t &valid_fields, const uint32_t field) {
  valid_fields &= ~field;
}

}  // namespace detail

inline void setPositionError(ControllerDebugCommon &debug,
                             const Eigen::Vector3d &value) {
  debug.position_error = value;
  detail::markField(debug.valid_fields, kPositionErrorValid);
}

inline bool hasPositionError(const ControllerDebugCommon &debug) {
  return detail::hasField(debug.valid_fields, kPositionErrorValid);
}

inline void clearPositionError(ControllerDebugCommon &debug) {
  debug.position_error.setZero();
  detail::clearField(debug.valid_fields, kPositionErrorValid);
}

inline void setVelocityError(ControllerDebugCommon &debug,
                             const Eigen::Vector3d &value) {
  debug.velocity_error = value;
  detail::markField(debug.valid_fields, kVelocityErrorValid);
}

inline bool hasVelocityError(const ControllerDebugCommon &debug) {
  return detail::hasField(debug.valid_fields, kVelocityErrorValid);
}

inline void clearVelocityError(ControllerDebugCommon &debug) {
  debug.velocity_error.setZero();
  detail::clearField(debug.valid_fields, kVelocityErrorValid);
}

inline void setYawError(ControllerDebugCommon &debug, const double value) {
  debug.yaw_error = value;
  detail::markField(debug.valid_fields, kYawErrorValid);
}

inline bool hasYawError(const ControllerDebugCommon &debug) {
  return detail::hasField(debug.valid_fields, kYawErrorValid);
}

inline void clearYawError(ControllerDebugCommon &debug) {
  debug.yaw_error = 0.0;
  detail::clearField(debug.valid_fields, kYawErrorValid);
}

inline void setYawRateError(ControllerDebugCommon &debug, const double value) {
  debug.yaw_rate_error = value;
  detail::markField(debug.valid_fields, kYawRateErrorValid);
}

inline bool hasYawRateError(const ControllerDebugCommon &debug) {
  return detail::hasField(debug.valid_fields, kYawRateErrorValid);
}

inline void clearYawRateError(ControllerDebugCommon &debug) {
  debug.yaw_rate_error = 0.0;
  detail::clearField(debug.valid_fields, kYawRateErrorValid);
}

inline void clearCommon(ControllerDebugCommon &debug) {
  debug.position_error.setZero();
  debug.velocity_error.setZero();
  debug.yaw_error = 0.0;
  debug.yaw_rate_error = 0.0;
  debug.valid_fields = 0u;
}

inline void setFeedbackAcceleration(Px4LocalControlDebug &debug,
                                    const Eigen::Vector3d &value) {
  debug.feedback_acceleration = value;
  detail::markField(debug.valid_fields, kFeedbackAccelerationValid);
}

inline bool hasFeedbackAcceleration(const Px4LocalControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kFeedbackAccelerationValid);
}

inline void clearFeedbackAcceleration(Px4LocalControlDebug &debug) {
  debug.feedback_acceleration.setZero();
  detail::clearField(debug.valid_fields, kFeedbackAccelerationValid);
}

inline void setCommandedAcceleration(Px4LocalControlDebug &debug,
                                     const Eigen::Vector3d &value) {
  debug.commanded_acceleration = value;
  detail::markField(debug.valid_fields, kCommandedAccelerationValid);
}

inline bool hasCommandedAcceleration(const Px4LocalControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kCommandedAccelerationValid);
}

inline void clearCommandedAcceleration(Px4LocalControlDebug &debug) {
  debug.commanded_acceleration.setZero();
  detail::clearField(debug.valid_fields, kCommandedAccelerationValid);
}

inline void setLimitedAcceleration(Px4LocalControlDebug &debug,
                                   const Eigen::Vector3d &value) {
  debug.limited_acceleration = value;
  detail::markField(debug.valid_fields, kLimitedAccelerationValid);
}

inline bool hasLimitedAcceleration(const Px4LocalControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kLimitedAccelerationValid);
}

inline void clearLimitedAcceleration(Px4LocalControlDebug &debug) {
  debug.limited_acceleration.setZero();
  detail::clearField(debug.valid_fields, kLimitedAccelerationValid);
}

inline void setCommandedYaw(Px4LocalControlDebug &debug, const double value) {
  debug.commanded_yaw = value;
  detail::markField(debug.valid_fields, kCommandedYawValid);
}

inline bool hasCommandedYaw(const Px4LocalControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kCommandedYawValid);
}

inline void clearCommandedYaw(Px4LocalControlDebug &debug) {
  debug.commanded_yaw = 0.0;
  detail::clearField(debug.valid_fields, kCommandedYawValid);
}

inline void setCommandedYawRate(Px4LocalControlDebug &debug,
                                const double value) {
  debug.commanded_yaw_rate = value;
  detail::markField(debug.valid_fields, kCommandedYawRateValid);
}

inline bool hasCommandedYawRate(const Px4LocalControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kCommandedYawRateValid);
}

inline void clearCommandedYawRate(Px4LocalControlDebug &debug) {
  debug.commanded_yaw_rate = 0.0;
  detail::clearField(debug.valid_fields, kCommandedYawRateValid);
}

inline void clearLocal(Px4LocalControlDebug &debug) {
  clearCommon(debug.common);
  debug.feedback_acceleration.setZero();
  debug.commanded_acceleration.setZero();
  debug.limited_acceleration.setZero();
  debug.commanded_yaw = 0.0;
  debug.commanded_yaw_rate = 0.0;
  debug.valid_fields = 0u;
}

inline void setReferenceAttitude(Px4AttitudeControlDebug &debug,
                                 const Eigen::Quaterniond &value) {
  debug.reference_attitude = value;
  detail::markField(debug.valid_fields, kReferenceAttitudeValid);
}

inline bool hasReferenceAttitude(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kReferenceAttitudeValid);
}

inline void clearReferenceAttitude(Px4AttitudeControlDebug &debug) {
  debug.reference_attitude = Eigen::Quaterniond::Identity();
  detail::clearField(debug.valid_fields, kReferenceAttitudeValid);
}

inline void setFeedbackAttitude(Px4AttitudeControlDebug &debug,
                                const Eigen::Quaterniond &value) {
  debug.feedback_attitude = value;
  detail::markField(debug.valid_fields, kFeedbackAttitudeValid);
}

inline bool hasFeedbackAttitude(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kFeedbackAttitudeValid);
}

inline void clearFeedbackAttitude(Px4AttitudeControlDebug &debug) {
  debug.feedback_attitude = Eigen::Quaterniond::Identity();
  detail::clearField(debug.valid_fields, kFeedbackAttitudeValid);
}

inline void setAttitudeError(Px4AttitudeControlDebug &debug,
                             const Eigen::Vector3d &value) {
  debug.attitude_error = value;
  detail::markField(debug.valid_fields, kAttitudeErrorValid);
}

inline bool hasAttitudeError(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kAttitudeErrorValid);
}

inline void clearAttitudeError(Px4AttitudeControlDebug &debug) {
  debug.attitude_error.setZero();
  detail::clearField(debug.valid_fields, kAttitudeErrorValid);
}

inline void setReferenceBodyRate(Px4AttitudeControlDebug &debug,
                                 const Eigen::Vector3d &value) {
  debug.reference_body_rate = value;
  detail::markField(debug.valid_fields, kReferenceBodyRateValid);
}

inline bool hasReferenceBodyRate(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kReferenceBodyRateValid);
}

inline void clearReferenceBodyRate(Px4AttitudeControlDebug &debug) {
  debug.reference_body_rate.setZero();
  detail::clearField(debug.valid_fields, kReferenceBodyRateValid);
}

inline void setFeedbackBodyRate(Px4AttitudeControlDebug &debug,
                                const Eigen::Vector3d &value) {
  debug.feedback_body_rate = value;
  detail::markField(debug.valid_fields, kFeedbackBodyRateValid);
}

inline bool hasFeedbackBodyRate(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kFeedbackBodyRateValid);
}

inline void clearFeedbackBodyRate(Px4AttitudeControlDebug &debug) {
  debug.feedback_body_rate.setZero();
  detail::clearField(debug.valid_fields, kFeedbackBodyRateValid);
}

inline void setCommandedBodyRate(Px4AttitudeControlDebug &debug,
                                 const Eigen::Vector3d &value) {
  debug.commanded_body_rate = value;
  detail::markField(debug.valid_fields, kCommandedBodyRateValid);
}

inline bool hasCommandedBodyRate(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kCommandedBodyRateValid);
}

inline void clearCommandedBodyRate(Px4AttitudeControlDebug &debug) {
  debug.commanded_body_rate.setZero();
  detail::clearField(debug.valid_fields, kCommandedBodyRateValid);
}

inline void setReferenceThrust(Px4AttitudeControlDebug &debug,
                               const double value) {
  debug.reference_thrust = value;
  detail::markField(debug.valid_fields, kReferenceThrustValid);
}

inline bool hasReferenceThrust(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kReferenceThrustValid);
}

inline void clearReferenceThrust(Px4AttitudeControlDebug &debug) {
  debug.reference_thrust = 0.0;
  detail::clearField(debug.valid_fields, kReferenceThrustValid);
}

inline void setCommandedThrust(Px4AttitudeControlDebug &debug,
                               const double value) {
  debug.commanded_thrust = value;
  detail::markField(debug.valid_fields, kCommandedThrustValid);
}

inline bool hasCommandedThrust(const Px4AttitudeControlDebug &debug) {
  return detail::hasField(debug.valid_fields, kCommandedThrustValid);
}

inline void clearCommandedThrust(Px4AttitudeControlDebug &debug) {
  debug.commanded_thrust = 0.0;
  detail::clearField(debug.valid_fields, kCommandedThrustValid);
}

inline void clearAttitude(Px4AttitudeControlDebug &debug) {
  clearCommon(debug.common);
  debug.reference_attitude = Eigen::Quaterniond::Identity();
  debug.feedback_attitude = Eigen::Quaterniond::Identity();
  debug.attitude_error.setZero();
  debug.reference_body_rate.setZero();
  debug.feedback_body_rate.setZero();
  debug.commanded_body_rate.setZero();
  debug.reference_thrust = 0.0;
  debug.commanded_thrust = 0.0;
  debug.valid_fields = 0u;
}

}  // namespace debug_utils
}  // namespace controller_data_types