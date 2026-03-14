#pragma once

#include <cmath>
#include <cstdint>

#include <Eigen/Dense>
#include <ros/duration.h>
#include <uav_control/TrajectoryPoint.h>

#include "utils/geometry_eigen_conversions.hpp"

namespace uav_control {
namespace trajectory_point_detail {

inline bool is_finite_vector(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline bool is_finite_quaternion(const Eigen::Quaterniond &q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z());
}

inline bool has_nonzero_vector(const Eigen::Vector3d &v, double eps = 1e-9) {
  return v.norm() > eps;
}

inline bool has_nonzero_quaternion(const Eigen::Quaterniond &q,
                                   double eps = 1e-9) {
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
  return std::abs(q.w() - identity.w()) > eps || std::abs(q.x()) > eps ||
         std::abs(q.y()) > eps || std::abs(q.z()) > eps;
}

} // namespace trajectory_point_detail

struct TrajectoryPointReference {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class Field : uint32_t {
    UNDEFINED = uav_control::TrajectoryPoint::FIELD_UNDEFINED,
    POSITION = uav_control::TrajectoryPoint::FIELD_POSITION,
    ORIENTATION = uav_control::TrajectoryPoint::FIELD_ORIENTATION,
    VELOCITY = uav_control::TrajectoryPoint::FIELD_VELOCITY,
    ACCELERATION = uav_control::TrajectoryPoint::FIELD_ACCELERATION,
    JERK = uav_control::TrajectoryPoint::FIELD_JERK,
    SNAP = uav_control::TrajectoryPoint::FIELD_SNAP,
    BODYRATES = uav_control::TrajectoryPoint::FIELD_BODYRATES,
    ANGULAR_ACCELERATION =
        uav_control::TrajectoryPoint::FIELD_ANGULAR_ACCELERATION,
    ANGULAR_JERK = uav_control::TrajectoryPoint::FIELD_ANGULAR_JERK,
    ANGULAR_SNAP = uav_control::TrajectoryPoint::FIELD_ANGULAR_SNAP,
    HEADING = uav_control::TrajectoryPoint::FIELD_HEADING,
    HEADING_RATE = uav_control::TrajectoryPoint::FIELD_HEADING_RATE,
    HEADING_ACCELERATION =
        uav_control::TrajectoryPoint::FIELD_HEADING_ACCELERATION,
    YAW = uav_control::TrajectoryPoint::FIELD_HEADING,
    YAW_RATE = uav_control::TrajectoryPoint::FIELD_HEADING_RATE,
    YAW_ACC = uav_control::TrajectoryPoint::FIELD_HEADING_ACCELERATION,
  };

  TrajectoryPointReference() = default;

  explicit TrajectoryPointReference(const uav_control::TrajectoryPoint &msg)
      : time_from_start(msg.time_from_start),
        position(geometryToEigen(msg.position)),
        orientation(geometryToEigen(msg.orientation)),
        velocity(geometryToEigen(msg.velocity)),
        acceleration(geometryToEigen(msg.acceleration)),
        jerk(geometryToEigen(msg.jerk)),
        snap(geometryToEigen(msg.snap)),
        bodyrates(geometryToEigen(msg.bodyrates)),
        angular_acceleration(geometryToEigen(msg.angular_acceleration)),
        angular_jerk(geometryToEigen(msg.angular_jerk)),
        angular_snap(geometryToEigen(msg.angular_snap)),
        heading(msg.heading),
        heading_rate(msg.heading_rate),
        heading_acceleration(msg.heading_acceleration),
        valid_mask(msg.valid_mask),
        trajectory_id(msg.trajectory_id) {
    if (trajectory_point_detail::is_finite_quaternion(orientation)) {
      const double qnorm = orientation.norm();
      if (qnorm > 1e-9) {
        orientation.normalize();
      }
    } else {
      orientation = Eigen::Quaterniond::Identity();
    }
  }

  uav_control::TrajectoryPoint toRosMessage() const {
    uav_control::TrajectoryPoint msg;
    msg.time_from_start = time_from_start;
    msg.position = vectorToPoint(eigenToGeometry(position));
    msg.orientation = eigenToGeometry(orientation);
    msg.velocity = eigenToGeometry(velocity);
    msg.acceleration = eigenToGeometry(acceleration);
    msg.jerk = eigenToGeometry(jerk);
    msg.snap = eigenToGeometry(snap);
    msg.bodyrates = eigenToGeometry(bodyrates);
    msg.angular_acceleration = eigenToGeometry(angular_acceleration);
    msg.angular_jerk = eigenToGeometry(angular_jerk);
    msg.angular_snap = eigenToGeometry(angular_snap);
    msg.heading = heading;
    msg.heading_rate = heading_rate;
    msg.heading_acceleration = heading_acceleration;
    msg.valid_mask = valid_mask;
    msg.trajectory_id = trajectory_id;
    return msg;
  }

  bool is_field_enabled(Field item) const {
    return (valid_mask & static_cast<uint32_t>(item)) != 0U;
  }

  void channel_enable(Field item) {
    valid_mask |= static_cast<uint32_t>(item);
  }

  void channel_disable(Field item) {
    valid_mask &= ~static_cast<uint32_t>(item);
  }

  void clear_all() {
    time_from_start = ros::Duration(0.0);
    position.setZero();
    orientation = Eigen::Quaterniond::Identity();
    velocity.setZero();
    acceleration.setZero();
    jerk.setZero();
    snap.setZero();
    bodyrates.setZero();
    angular_acceleration.setZero();
    angular_jerk.setZero();
    angular_snap.setZero();
    heading = 0.0;
    heading_rate = 0.0;
    heading_acceleration = 0.0;
    valid_mask = static_cast<uint32_t>(Field::UNDEFINED);
    trajectory_id = 0U;
  }

  void infer_valid_mask_from_nonzero() {
    valid_mask = static_cast<uint32_t>(Field::UNDEFINED);
    if (trajectory_point_detail::has_nonzero_vector(position)) {
      channel_enable(Field::POSITION);
    }
    if (trajectory_point_detail::has_nonzero_quaternion(orientation)) {
      channel_enable(Field::ORIENTATION);
    }
    if (trajectory_point_detail::has_nonzero_vector(velocity)) {
      channel_enable(Field::VELOCITY);
    }
    if (trajectory_point_detail::has_nonzero_vector(acceleration)) {
      channel_enable(Field::ACCELERATION);
    }
    if (trajectory_point_detail::has_nonzero_vector(jerk)) {
      channel_enable(Field::JERK);
    }
    if (trajectory_point_detail::has_nonzero_vector(snap)) {
      channel_enable(Field::SNAP);
    }
    if (trajectory_point_detail::has_nonzero_vector(bodyrates)) {
      channel_enable(Field::BODYRATES);
    }
    if (trajectory_point_detail::has_nonzero_vector(angular_acceleration)) {
      channel_enable(Field::ANGULAR_ACCELERATION);
    }
    if (trajectory_point_detail::has_nonzero_vector(angular_jerk)) {
      channel_enable(Field::ANGULAR_JERK);
    }
    if (trajectory_point_detail::has_nonzero_vector(angular_snap)) {
      channel_enable(Field::ANGULAR_SNAP);
    }
    if (std::abs(heading) > 1e-9) {
      channel_enable(Field::HEADING);
    }
    if (std::abs(heading_rate) > 1e-9) {
      channel_enable(Field::HEADING_RATE);
    }
    if (std::abs(heading_acceleration) > 1e-9) {
      channel_enable(Field::HEADING_ACCELERATION);
    }
  }

  void set_position(const Eigen::Vector3d &value) {
    position = value;
    channel_enable(Field::POSITION);
  }

  void set_orientation(const Eigen::Quaterniond &value) {
    orientation = value;
    if (trajectory_point_detail::is_finite_quaternion(orientation) &&
        orientation.norm() > 1e-9) {
      orientation.normalize();
    } else {
      orientation = Eigen::Quaterniond::Identity();
    }
    channel_enable(Field::ORIENTATION);
  }

  void set_velocity(const Eigen::Vector3d &value) {
    velocity = value;
    channel_enable(Field::VELOCITY);
  }

  void set_acceleration(const Eigen::Vector3d &value) {
    acceleration = value;
    channel_enable(Field::ACCELERATION);
  }

  void set_jerk(const Eigen::Vector3d &value) {
    jerk = value;
    channel_enable(Field::JERK);
  }

  void set_snap(const Eigen::Vector3d &value) {
    snap = value;
    channel_enable(Field::SNAP);
  }

  void set_bodyrates(const Eigen::Vector3d &value) {
    bodyrates = value;
    channel_enable(Field::BODYRATES);
  }

  void set_yaw(double value) {
    heading = value;
    channel_enable(Field::HEADING);
  }

  void set_yaw_rate(double value) {
    heading_rate = value;
    channel_enable(Field::HEADING_RATE);
  }

  void set_yaw_acc(double value) {
    heading_acceleration = value;
    channel_enable(Field::HEADING_ACCELERATION);
  }

  ros::Duration time_from_start{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d snap{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bodyrates{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_jerk{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_snap{Eigen::Vector3d::Zero()};
  double heading{0.0};
  double heading_rate{0.0};
  double heading_acceleration{0.0};
  uint32_t valid_mask{static_cast<uint32_t>(Field::UNDEFINED)};
  uint32_t trajectory_id{0U};
};

} // namespace uav_control
