#pragma once

#include "sunray_helper/sunray_helper.h"
#include "sunray_helper/sunray_robot_group.h"

#include <control_msg_types/trajectory_point.hpp>
#include <uav_control/Trajectory.h>

#include <Eigen/Dense>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace sunray_demo {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNominalSpeedMps = 0.6;
constexpr double kTimeoutMarginS = 15.0;
constexpr double kPositionToleranceM = 0.15;
constexpr double kStatePollHz = 20.0;
constexpr int kCircleSegments = 48;

inline Eigen::Vector3d offset_xy(const Eigen::Vector3d &origin, double x,
                                 double y) {
  return origin + Eigen::Vector3d(x, y, 0.0);
}

inline double segment_duration_s(const Eigen::Vector3d &from,
                                 const Eigen::Vector3d &to, double speed_mps) {
  const double speed = (speed_mps > 1e-6) ? speed_mps : kNominalSpeedMps;
  return std::max(0.1, (to - from).norm() / speed);
}

inline double path_timeout_s(const Eigen::Vector3d &start,
                             const std::vector<Eigen::Vector3d> &waypoints,
                             double speed_mps = kNominalSpeedMps,
                             double extra_s = kTimeoutMarginS) {
  Eigen::Vector3d last = start;
  double total = 0.0;
  for (const auto &point : waypoints) {
    total += segment_duration_s(last, point, speed_mps);
    last = point;
  }
  return total + extra_s;
}

inline double path_timeout_s(
    const Eigen::Vector3d &start,
    const std::vector<std::pair<Eigen::Vector3d, double>> &waypoints,
    double default_speed_mps = kNominalSpeedMps,
    double extra_s = kTimeoutMarginS) {
  Eigen::Vector3d last = start;
  double total = 0.0;
  for (const auto &point : waypoints) {
    const double speed = (point.second > 1e-6) ? point.second : default_speed_mps;
    total += segment_duration_s(last, point.first, speed);
    last = point.first;
  }
  return total + extra_s;
}

inline uav_control::Trajectory
make_trajectory(const Eigen::Vector3d &start,
                const std::vector<Eigen::Vector3d> &waypoints,
                double speed_mps = kNominalSpeedMps) {
  uav_control::Trajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.reference_type = uav_control::Trajectory::FLAT_OUTPUT;
  trajectory.flat_output_order = uav_control::Trajectory::ACCELERATION;

  Eigen::Vector3d last = start;
  double total = 0.0;
  for (const auto &point : waypoints) {
    total += segment_duration_s(last, point, speed_mps);
    uav_control::TrajectoryPointReference ref;
    ref.clear_all();
    ref.time_from_start = ros::Duration(total);
    ref.set_position(point);
    trajectory.points.push_back(ref.toRosMessage());
    last = point;
  }
  return trajectory;
}

inline std::vector<Eigen::Vector3d> circle_waypoints(
    const Eigen::Vector3d &center, double radius = 1.0,
    int segments = kCircleSegments) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(std::max(1, segments));
  for (int i = 1; i <= std::max(1, segments); ++i) {
    const double theta = kPi + (2.0 * kPi * static_cast<double>(i) /
                                static_cast<double>(std::max(1, segments)));
    points.push_back(center +
                     Eigen::Vector3d(radius * std::cos(theta),
                                     radius * std::sin(theta), 0.0));
  }
  return points;
}

inline std::vector<Eigen::Vector3d> square_waypoints(const Eigen::Vector3d &center,
                                                     double half_side = 1.0) {
  return {
      offset_xy(center, 1.0 * half_side, 1.0 * half_side),
      offset_xy(center, 1.0 * half_side, -1.0 * half_side),
      offset_xy(center, -1.0 * half_side, -1.0 * half_side),
      offset_xy(center, -1.0 * half_side, 1.0 * half_side),
  };
}

inline std::vector<Eigen::Vector3d>
pentagram_waypoints(const Eigen::Vector3d &first_vertex, double radius = 1.0) {
  const Eigen::Vector3d center = first_vertex + Eigen::Vector3d(0.0, -radius, 0.0);
  std::vector<Eigen::Vector3d> vertices(5);
  for (int i = 0; i < 5; ++i) {
    const double theta = kPi / 2.0 + 2.0 * kPi * static_cast<double>(i) / 5.0;
    vertices[i] =
        center + Eigen::Vector3d(radius * std::cos(theta), radius * std::sin(theta), 0.0);
  }
  return {vertices[2], vertices[4], vertices[1], vertices[3], vertices[0]};
}

inline bool wait_for_state(Sunray_Helper &helper,
                           sunray_fsm::SunrayState expected_state,
                           double timeout_s) {
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(std::max(0.0, timeout_s));
  ros::Rate rate(kStatePollHz);
  while (ros::ok() && ros::WallTime::now() <= deadline) {
    if (helper.get_statemachine_state() == expected_state) {
      return true;
    }
    rate.sleep();
  }
  return false;
}

inline bool wait_for_position(Sunray_Helper &helper,
                              const Eigen::Vector3d &target_position,
                              double timeout_s,
                              double tolerance_m = kPositionToleranceM) {
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(std::max(0.0, timeout_s));
  ros::Rate rate(kStatePollHz);
  while (ros::ok() && ros::WallTime::now() <= deadline) {
    if ((helper.get_uav_position() - target_position).norm() <= tolerance_m) {
      return true;
    }
    rate.sleep();
  }
  return false;
}

inline bool wait_for_hover_and_position(Sunray_Helper &helper,
                                        const Eigen::Vector3d &target_position,
                                        double timeout_s) {
  return wait_for_state(helper, sunray_fsm::SunrayState::HOVER, timeout_s) &&
         wait_for_position(helper, target_position, 2.0);
}

inline bool run_trajectory(Sunray_Helper &helper,
                           const uav_control::Trajectory &trajectory) {
  if (trajectory.points.empty()) {
    return false;
  }
  const uav_control::TrajectoryPointReference tail(trajectory.points.back());
  return helper.set_trajectory_async(trajectory) &&
         wait_for_hover_and_position(
             helper, tail.position,
             trajectory.points.back().time_from_start.toSec() + kTimeoutMarginS);
}

inline bool run_trajectory(sunray_helper_fluent::Sunray_RobotGroup &robot_group,
                           const uav_control::Trajectory &trajectory) {
  if (trajectory.points.empty()) {
    return false;
  }
  const uav_control::TrajectoryPointReference tail(trajectory.points.back());
  return robot_group
             .follow_trajectory(
                 trajectory,
                 trajectory.points.back().time_from_start.toSec() +
                     kTimeoutMarginS)
             .wait_for_completed() &&
         wait_for_position(robot_group.raw_helper(), tail.position, 8.0);
}

inline int fail(const char *tag, const char *step) {
  ROS_WARN("[%s] %s failed", tag, step);
  ros::shutdown();
  return 1;
}

inline int done(const char *tag, const char *message) {
  ROS_INFO("[%s] %s", tag, message);
  ros::shutdown();
  return 0;
}

} // namespace sunray_demo
