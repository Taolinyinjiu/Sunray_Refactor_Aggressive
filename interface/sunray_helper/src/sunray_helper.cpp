#include "sunray_helper/sunray_helper.h"

#include <Eigen/Geometry>

SunrayHelper::SunrayHelper(ros::NodeHandle &nh) : nh_(nh) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "idle";
  current_uav_info_.latest_log = "sunray_helper initialized";
  current_uav_info_.active_task = "none";
}

SunrayHelper::~SunrayHelper() = default;

SunrayUavInfo SunrayHelper::get_uav_info() const { return current_uav_info_; }

sunray_common::QuadStateEstimate SunrayHelper::get_odom() const {
  return current_uav_info_.odom;
}

bool SunrayHelper::takeoff(std::optional<double>, std::optional<double>) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "takeoff";
  current_uav_info_.latest_log = "takeoff command accepted";
  current_uav_info_.active_task = "takeoff";
  current_uav_info_.task_completed = false;
  current_uav_info_.task_succeeded = false;
  current_uav_info_.task_progress = 0.0;
  return true;
}

bool SunrayHelper::takeoff_wait_for_complete(std::optional<double>,
                                             std::optional<double>) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "hover";
  current_uav_info_.latest_log = "takeoff completed";
  current_uav_info_.active_task = "takeoff";
  current_uav_info_.task_completed = true;
  current_uav_info_.task_succeeded = true;
  current_uav_info_.task_progress = 1.0;
  return true;
}

bool SunrayHelper::land(std::optional<int>, std::optional<double>) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "landing";
  current_uav_info_.latest_log = "land command accepted";
  current_uav_info_.active_task = "land";
  current_uav_info_.task_completed = false;
  current_uav_info_.task_succeeded = false;
  current_uav_info_.task_progress = 0.0;
  return true;
}

bool SunrayHelper::land_wait_for_complete(std::optional<int>,
                                          std::optional<double>) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "landed";
  current_uav_info_.latest_log = "land completed";
  current_uav_info_.active_task = "land";
  current_uav_info_.task_completed = true;
  current_uav_info_.task_succeeded = true;
  current_uav_info_.task_progress = 1.0;
  return true;
}

bool SunrayHelper::return_home(std::optional<Eigen::Vector3d>,
                               std::optional<double>,
                               std::optional<int>,
                               std::optional<double>) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "return_home";
  current_uav_info_.latest_log = "return_home command accepted";
  current_uav_info_.active_task = "return_home";
  current_uav_info_.task_completed = false;
  current_uav_info_.task_succeeded = false;
  current_uav_info_.task_progress = 0.0;
  return true;
}

bool SunrayHelper::return_home_wait_for_complete(std::optional<Eigen::Vector3d>,
                                                 std::optional<double>,
                                                 std::optional<int>,
                                                 std::optional<double>) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "landed";
  current_uav_info_.latest_log = "return_home completed";
  current_uav_info_.active_task = "return_home";
  current_uav_info_.task_completed = true;
  current_uav_info_.task_succeeded = true;
  current_uav_info_.task_progress = 1.0;
  return true;
}

bool SunrayHelper::move_to(const MovePoint &point) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "move_to";
  current_uav_info_.latest_log = "move_to command accepted";
  current_uav_info_.active_task = "move_to";
  current_uav_info_.odom.position = point.position;
  if (point.velocity.has_value()) {
    current_uav_info_.odom.velocity =
        Eigen::Vector3d::Constant(point.velocity.value());
  }
  if (point.yaw.has_value()) {
    current_uav_info_.odom.orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(point.yaw.value(),
                                             Eigen::Vector3d::UnitZ()));
  }
  if (point.yaw_rate.has_value()) {
    current_uav_info_.odom.bodyrates.z() = point.yaw_rate.value();
  }
  current_uav_info_.task_completed = false;
  current_uav_info_.task_succeeded = false;
  current_uav_info_.task_progress = 0.0;
  return true;
}

bool SunrayHelper::move_to(Eigen::Vector3d position,
                           std::optional<double> velocity,
                           std::optional<double> yaw,
                           std::optional<double> yaw_rate) {
  MovePoint point;
  point.position = position;
  point.velocity = velocity;
  point.yaw = yaw;
  point.yaw_rate = yaw_rate;
  return move_to(point);
}

bool SunrayHelper::move_to_wait_for_complete(const MovePoint &point) {
  move_to(point);
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "hover";
  current_uav_info_.latest_log = "move_to completed";
  current_uav_info_.task_completed = true;
  current_uav_info_.task_succeeded = true;
  current_uav_info_.task_progress = 1.0;
  return true;
}

bool SunrayHelper::move_to_wait_for_complete(Eigen::Vector3d position,
                                             std::optional<double> velocity,
                                             std::optional<double> yaw,
                                             std::optional<double> yaw_rate) {
  MovePoint point;
  point.position = position;
  point.velocity = velocity;
  point.yaw = yaw;
  point.yaw_rate = yaw_rate;
  return move_to_wait_for_complete(point);
}

bool SunrayHelper::set_yaw(double yaw, bool relative) {
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "set_yaw";
  current_uav_info_.latest_log = "set_yaw command accepted";
  current_uav_info_.active_task = "set_yaw";

  const double target_yaw = relative ? current_uav_info_.odom.getYaw() + yaw : yaw;
  current_uav_info_.odom.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(target_yaw, Eigen::Vector3d::UnitZ()));
  current_uav_info_.task_completed = false;
  current_uav_info_.task_succeeded = false;
  current_uav_info_.task_progress = 0.0;
  return true;
}

bool SunrayHelper::set_yaw_wait_for_complete(double yaw, bool relative) {
  set_yaw(yaw, relative);
  current_uav_info_.stamp = ros::Time::now();
  current_uav_info_.state_machine_state = "hover";
  current_uav_info_.latest_log = "set_yaw completed";
  current_uav_info_.task_completed = true;
  current_uav_info_.task_succeeded = true;
  current_uav_info_.task_progress = 1.0;
  return true;
}
