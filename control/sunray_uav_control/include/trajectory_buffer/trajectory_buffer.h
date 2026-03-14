#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <ros/time.h>
#include <uav_control/Trajectory.h>
#include <uav_control/TrajectoryPoint.h>

#include "control_msg_types/trajectory_point.hpp"

namespace uav_control {

struct TrajectoryBundle {
  uint32_t trajectory_id{0U};
  uint8_t reference_type{uav_control::Trajectory::FLAT_OUTPUT};
  uint8_t flat_output_order{uav_control::Trajectory::ACCELERATION};
  ros::Time receive_time{};
  ros::Time effective_time{};
  std::vector<uav_control::TrajectoryPoint> points{};
};

struct TrajectorySample {
  bool valid{false};
  bool reached_end{false};
  uint32_t trajectory_id{0U};
  uav_control::TrajectoryPoint point{};
};

enum class TrajectoryBufferStatus : uint8_t {
  EMPTY = 0,
  READY = 1,
  COMPLETED = 2,
  ABORT = 3,
  IMPOSSIBLE = 4,
};

class TrajectoryBufferManager {
public:
  void clear() {
    pending_.clear();
    active_valid_ = false;
    active_ = TrajectoryBundle();
    status_ = TrajectoryBufferStatus::EMPTY;
    active_id_ = 0U;
    last_received_id_ = 0U;
  }

  bool has_reference() const { return active_valid_ || !pending_.empty(); }

  bool has_active() const { return active_valid_; }

  TrajectoryBufferStatus status() const { return status_; }

  uint32_t active_id() const { return active_id_; }

  void mark_completed() { status_ = TrajectoryBufferStatus::COMPLETED; }

  void mark_abort() { status_ = TrajectoryBufferStatus::ABORT; }

  bool push_pending(const uav_control::Trajectory &trajectory,
                    const ros::Time &receive_time,
                    uint32_t fallback_trajectory_id = 0U) {
    TrajectoryBundle bundle;
    if (!build_bundle(trajectory, receive_time, fallback_trajectory_id, &bundle)) {
      status_ = TrajectoryBufferStatus::IMPOSSIBLE;
      return false;
    }

    if (bundle.trajectory_id <= last_received_id_ &&
        bundle.trajectory_id != 0U) {
      return false;
    }

    pending_.clear();
    if (bundle.effective_time <= receive_time || !active_valid_) {
      active_ = bundle;
      active_valid_ = true;
      active_id_ = bundle.trajectory_id;
      status_ = TrajectoryBufferStatus::READY;
    } else {
      pending_.push_back(bundle);
    }
    last_received_id_ = bundle.trajectory_id;
    return true;
  }

  void update_active(const ros::Time &now) {
    while (!pending_.empty()) {
      const TrajectoryBundle &candidate = pending_.front();
      if (candidate.effective_time > now) {
        break;
      }
      active_ = candidate;
      active_valid_ = true;
      active_id_ = candidate.trajectory_id;
      status_ = TrajectoryBufferStatus::READY;
      pending_.pop_front();
    }
  }

  TrajectorySample sample(const ros::Time &now) const {
    TrajectorySample sample;
    if (!active_valid_ || active_.points.empty()) {
      return sample;
    }

    if (now < active_.effective_time) {
      return sample;
    }

    const double t = (now - active_.effective_time).toSec();
    if (t < 0.0) {
      return sample;
    }

    const auto normalized_last = normalize_point(active_.points.back());
    const double t_end = normalized_last.time_from_start.toSec();
    if (active_.points.size() == 1U || t >= t_end) {
      sample.valid = true;
      sample.reached_end = (active_.points.size() > 1U);
      sample.trajectory_id = active_.trajectory_id;
      sample.point = normalized_last.toRosMessage();
      return sample;
    }

    size_t seg_idx = 0U;
    while (seg_idx + 1U < active_.points.size()) {
      const auto next_ref = normalize_point(active_.points[seg_idx + 1U]);
      if (next_ref.time_from_start.toSec() >= t) {
        break;
      }
      ++seg_idx;
    }

    if (seg_idx + 1U >= active_.points.size()) {
      sample.valid = true;
      sample.reached_end = true;
      sample.trajectory_id = active_.trajectory_id;
      sample.point = normalized_last.toRosMessage();
      return sample;
    }

    const auto p0 = normalize_point(active_.points[seg_idx]);
    const auto p1 = normalize_point(active_.points[seg_idx + 1U]);
    const double t0 = p0.time_from_start.toSec();
    const double t1 = p1.time_from_start.toSec();
    const double u = (t1 > t0) ? clamp01((t - t0) / (t1 - t0)) : 0.0;

    TrajectoryPointReference out;
    out.clear_all();
    out.time_from_start = ros::Duration(std::max(0.0, t));
    out.trajectory_id = active_.trajectory_id;

    interpolate_vector_field(p0, p1, u, TrajectoryPointReference::Field::POSITION,
                             &TrajectoryPointReference::position, &out,
                             &TrajectoryPointReference::set_position);
    interpolate_orientation_field(p0, p1, u, &out);
    interpolate_vector_field(
        p0, p1, u, TrajectoryPointReference::Field::VELOCITY,
        &TrajectoryPointReference::velocity, &out,
        &TrajectoryPointReference::set_velocity);
    interpolate_vector_field(
        p0, p1, u, TrajectoryPointReference::Field::ACCELERATION,
        &TrajectoryPointReference::acceleration, &out,
        &TrajectoryPointReference::set_acceleration);
    interpolate_vector_field(p0, p1, u, TrajectoryPointReference::Field::JERK,
                             &TrajectoryPointReference::jerk, &out,
                             &TrajectoryPointReference::set_jerk);
    interpolate_vector_field(p0, p1, u, TrajectoryPointReference::Field::SNAP,
                             &TrajectoryPointReference::snap, &out,
                             &TrajectoryPointReference::set_snap);
    interpolate_vector_field(
        p0, p1, u, TrajectoryPointReference::Field::BODYRATES,
        &TrajectoryPointReference::bodyrates, &out,
        &TrajectoryPointReference::set_bodyrates);
    interpolate_scalar_field(p0, p1, u, TrajectoryPointReference::Field::HEADING,
                             &TrajectoryPointReference::heading, &out,
                             &TrajectoryPointReference::set_yaw);
    interpolate_scalar_field(
        p0, p1, u, TrajectoryPointReference::Field::HEADING_RATE,
        &TrajectoryPointReference::heading_rate, &out,
        &TrajectoryPointReference::set_yaw_rate);
    interpolate_scalar_field(
        p0, p1, u, TrajectoryPointReference::Field::HEADING_ACCELERATION,
        &TrajectoryPointReference::heading_acceleration, &out,
        &TrajectoryPointReference::set_yaw_acc);

    sample.valid = true;
    sample.trajectory_id = active_.trajectory_id;
    sample.point = out.toRosMessage();
    return sample;
  }

private:
  static bool build_bundle(const uav_control::Trajectory &trajectory,
                           const ros::Time &receive_time,
                           uint32_t fallback_trajectory_id,
                           TrajectoryBundle *bundle) {
    if (!bundle || trajectory.points.empty()) {
      return false;
    }

    TrajectoryBundle out;
    out.reference_type = trajectory.reference_type;
    out.flat_output_order = trajectory.flat_output_order;
    out.receive_time = receive_time;
    out.effective_time = trajectory.header.stamp.isZero()
                             ? receive_time
                             : trajectory.header.stamp;
    out.points = trajectory.points;
    out.trajectory_id = resolve_trajectory_id(trajectory, fallback_trajectory_id);

    if (!validate_basic(out)) {
      return false;
    }
    *bundle = out;
    return true;
  }

  static uint32_t resolve_trajectory_id(const uav_control::Trajectory &trajectory,
                                        uint32_t fallback_trajectory_id) {
    for (const auto &point : trajectory.points) {
      if (point.trajectory_id != 0U) {
        return point.trajectory_id;
      }
    }
    return fallback_trajectory_id;
  }

  static bool validate_basic(const TrajectoryBundle &bundle) {
    if (bundle.points.empty()) {
      return false;
    }

    double last_t = -1e9;
    for (const auto &point : bundle.points) {
      const TrajectoryPointReference ref = normalize_point(point);
      const double t = ref.time_from_start.toSec();
      if (t < 0.0 || t < last_t) {
        return false;
      }
      last_t = t;
    }
    return true;
  }

  static TrajectoryPointReference normalize_point(
      const uav_control::TrajectoryPoint &point) {
    TrajectoryPointReference ref(point);
    if (ref.valid_mask ==
        static_cast<uint32_t>(TrajectoryPointReference::Field::UNDEFINED)) {
      ref.infer_valid_mask_from_nonzero();
    }
    return ref;
  }

  static double clamp01(double x) {
    return std::max(0.0, std::min(1.0, x));
  }

  static Eigen::Vector3d lerp_vector(const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b, double u) {
    return (1.0 - u) * a + u * b;
  }

  static double lerp_scalar(double a, double b, double u) {
    return (1.0 - u) * a + u * b;
  }

  template <typename Setter>
  static void interpolate_vector_field(
      const TrajectoryPointReference &p0, const TrajectoryPointReference &p1,
      double u, TrajectoryPointReference::Field field,
      Eigen::Vector3d TrajectoryPointReference::*member,
      TrajectoryPointReference *out, Setter setter) {
    if (!out) {
      return;
    }

    const bool has0 = p0.is_field_enabled(field);
    const bool has1 = p1.is_field_enabled(field);
    if (!has0 && !has1) {
      return;
    }

    if (has0 && has1) {
      (out->*setter)(lerp_vector(p0.*member, p1.*member, u));
      return;
    }

    (out->*setter)(has0 ? (p0.*member) : (p1.*member));
  }

  template <typename Setter>
  static void interpolate_scalar_field(
      const TrajectoryPointReference &p0, const TrajectoryPointReference &p1,
      double u, TrajectoryPointReference::Field field,
      double TrajectoryPointReference::*member, TrajectoryPointReference *out,
      Setter setter) {
    if (!out) {
      return;
    }

    const bool has0 = p0.is_field_enabled(field);
    const bool has1 = p1.is_field_enabled(field);
    if (!has0 && !has1) {
      return;
    }

    if (has0 && has1) {
      (out->*setter)(lerp_scalar(p0.*member, p1.*member, u));
      return;
    }

    (out->*setter)(has0 ? (p0.*member) : (p1.*member));
  }

  static void interpolate_orientation_field(
      const TrajectoryPointReference &p0, const TrajectoryPointReference &p1,
      double u, TrajectoryPointReference *out) {
    if (!out) {
      return;
    }

    const bool has0 =
        p0.is_field_enabled(TrajectoryPointReference::Field::ORIENTATION);
    const bool has1 =
        p1.is_field_enabled(TrajectoryPointReference::Field::ORIENTATION);
    if (!has0 && !has1) {
      return;
    }
    if (has0 && has1) {
      out->set_orientation(p0.orientation.slerp(u, p1.orientation));
      return;
    }
    out->set_orientation(has0 ? p0.orientation : p1.orientation);
  }

  std::deque<TrajectoryBundle> pending_{};
  TrajectoryBundle active_{};
  bool active_valid_{false};
  TrajectoryBufferStatus status_{TrajectoryBufferStatus::EMPTY};
  uint32_t active_id_{0U};
  uint32_t last_received_id_{0U};
};

} // namespace uav_control
