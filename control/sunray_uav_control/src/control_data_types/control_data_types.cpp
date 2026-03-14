#include "control_data_types/control_data_types.h"

namespace uav_control {

void ControllerOutput::channel_enable(ControllerOutputMask item) {
  output_mask |= static_cast<uint32_t>(item);
}

void ControllerOutput::channel_disable(ControllerOutputMask item) {
  output_mask &= ~static_cast<uint32_t>(item);
}

bool ControllerOutput::is_channel_enabled(ControllerOutputMask item) const {
  return (output_mask & static_cast<uint32_t>(item)) != 0U;
}

void ControllerOutput::clear_all(void) {
  output_mask = static_cast<uint32_t>(ControllerOutputMask::UNDEFINED);
  position = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  acceleration_or_force = Eigen::Vector3d::Zero();
  yaw = 0.0;
  yaw_rate = 0.0;
  attitude = Eigen::Quaterniond::Identity();
  body_rate = Eigen::Vector3d::Zero();
  thrust = 0.0;
}

} // namespace uav_control
