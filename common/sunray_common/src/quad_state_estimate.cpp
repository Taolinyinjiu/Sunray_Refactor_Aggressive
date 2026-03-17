#include "sunray_common/quad_state_estimate.h"

#include <cmath>

#include "sunray_common/geometry_eigen_conversions.h"

namespace sunray_common {

QuadStateEstimate::QuadStateEstimate()
    : timestamp(0.0),
      position(Eigen::Vector3d::Zero()),
      velocity(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()),
      bodyrates(Eigen::Vector3d::Zero()) {}

QuadStateEstimate::QuadStateEstimate(
    const nav_msgs::Odometry &state_estimate_msg) {
  timestamp = state_estimate_msg.header.stamp;
  position = geometryToEigen(state_estimate_msg.pose.pose.position);
  velocity = geometryToEigen(state_estimate_msg.twist.twist.linear);
  orientation = geometryToEigen(state_estimate_msg.pose.pose.orientation);
  bodyrates = geometryToEigen(state_estimate_msg.twist.twist.angular);
}

nav_msgs::Odometry QuadStateEstimate::toRosMessage() const {
  nav_msgs::Odometry msg;

  msg.header.stamp = timestamp;

  msg.child_frame_id = "body";
  msg.pose.pose.position = vectorToPoint(eigenToGeometry(position));
  msg.twist.twist.linear = eigenToGeometry(velocity);
  msg.pose.pose.orientation = eigenToGeometry(orientation);
  msg.twist.twist.angular = eigenToGeometry(bodyrates);

  return msg;
}

void QuadStateEstimate::transform_VelocityToWorldFrame() {
  velocity = orientation * velocity;
}

double QuadStateEstimate::getYaw() const {
  const Eigen::Quaterniond normalized_orientation = orientation.normalized();
  const double siny_cosp =
      2.0 * (normalized_orientation.w() * normalized_orientation.z() +
             normalized_orientation.x() * normalized_orientation.y());
  const double cosy_cosp =
      1.0 - 2.0 * (normalized_orientation.y() * normalized_orientation.y() +
                   normalized_orientation.z() * normalized_orientation.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

bool QuadStateEstimate::isValid() const {
  if (std::isnan(position.norm())) {
    return false;
  }
  if (std::isnan(velocity.norm())) {
    return false;
  }
  if (std::isnan(orientation.norm())) {
    return false;
  }
  if (std::isnan(bodyrates.norm())) {
    return false;
  }

  return true;
}

} // namespace sunray_common
