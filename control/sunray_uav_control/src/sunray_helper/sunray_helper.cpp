#include "sunray_helper/sunray_helper.h"

#include <algorithm>
#include <atomic>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <cmath>

#include <uav_control/AttitudeCmd.h>
#include <uav_control/AttitudeCmdEnvelope.h>
#include <uav_control/ComplexCmd.h>
#include <uav_control/ComplexCmdEnvelope.h>
#include <uav_control/ControlMeta.h>
#include <uav_control/Land.h>
#include <uav_control/PositionRequest.h>
#include <uav_control/ReturnHome.h>
#include <uav_control/Takeoff.h>
#include <uav_control/Trajectory.h>
#include <uav_control/TrajectoryEnvelope.h>
#include <uav_control/VelocityCmd.h>
#include <uav_control/VelocityCmdEnvelope.h>

namespace {
constexpr double kMinTrajectorySegmentTimeS = 0.1;
constexpr double kTrajectoryMetaTimeoutMarginS = 1.0;

std::string normalize_ns(const std::string &ns) {
  if (!ns.empty() && ns.front() == '/') {
    return ns.substr(1);
  }
  return ns;
}

std::string resolve_uav_namespace(ros::NodeHandle &nh) {
  std::string key;
  std::string ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, ns) && !ns.empty()) {
    return normalize_ns(ns);
  }

  std::string name;
  int id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh.searchParam("uav_name", key)) {
    ok_name = nh.getParam(key, name) && !name.empty();
  }
  if (nh.searchParam("uav_id", key)) {
    ok_id = nh.getParam(key, id);
  }
  if (ok_name && ok_id) {
    return name + std::to_string(id);
  }

  // fallback to global params
  name = "uav";
  id = 1;
  nh.param("/uav_name", name, name);
  nh.param("/uav_id", id, id);
  return name + std::to_string(id);
}

template <typename T>
void load_helper_param(const ros::NodeHandle &local_nh,
                       const ros::NodeHandle *helper_cfg_nh,
                       const ros::NodeHandle *uav_cfg_nh,
                       const std::string &name, T *value) {
  if (!value) {
    return;
  }
  if (local_nh.getParam(name, *value)) {
    return;
  }
  if (helper_cfg_nh && helper_cfg_nh->getParam(name, *value)) {
    return;
  }
  if (uav_cfg_nh) {
    (void)uav_cfg_nh->getParam(name, *value);
  }
}

Eigen::Quaterniond quat_from_yaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

double yaw_from_quat(const Eigen::Quaterniond &q) {
  Eigen::Quaterniond normalized = q;
  if (!std::isfinite(normalized.w()) || !std::isfinite(normalized.x()) ||
      !std::isfinite(normalized.y()) || !std::isfinite(normalized.z()) ||
      normalized.norm() < 1e-9) {
    normalized = Eigen::Quaterniond::Identity();
  } else {
    normalized.normalize();
  }

  const double siny_cosp =
      2.0 * (normalized.w() * normalized.z() +
             normalized.x() * normalized.y());
  const double cosy_cosp =
      1.0 - 2.0 * (normalized.y() * normalized.y() +
                   normalized.z() * normalized.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Vector3d rpy_from_quat(const Eigen::Quaterniond &q) {
  Eigen::Quaterniond normalized = q;
  if (!std::isfinite(normalized.w()) || !std::isfinite(normalized.x()) ||
      !std::isfinite(normalized.y()) || !std::isfinite(normalized.z()) ||
      normalized.norm() < 1e-9) {
    normalized = Eigen::Quaterniond::Identity();
  } else {
    normalized.normalize();
  }

  const double sinr_cosp =
      2.0 * (normalized.w() * normalized.x() +
             normalized.y() * normalized.z());
  const double cosr_cosp =
      1.0 - 2.0 * (normalized.x() * normalized.x() +
                   normalized.y() * normalized.y());
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp =
      2.0 * (normalized.w() * normalized.y() -
             normalized.z() * normalized.x());
  const double pitch =
      std::asin(std::max(-1.0, std::min(1.0, sinp)));

  const double siny_cosp =
      2.0 * (normalized.w() * normalized.z() +
             normalized.x() * normalized.y());
  const double cosy_cosp =
      1.0 - 2.0 * (normalized.y() * normalized.y() +
                   normalized.z() * normalized.z());
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

double wrap_to_pi(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double angular_distance(double lhs, double rhs) {
  return std::abs(wrap_to_pi(lhs - rhs));
}

const char *fsm_state_to_string(sunray_fsm::SunrayState state) {
  switch (state) {
  case sunray_fsm::SunrayState::OFF:
    return "OFF";
  case sunray_fsm::SunrayState::TAKEOFF:
    return "TAKEOFF";
  case sunray_fsm::SunrayState::HOVER:
    return "HOVER";
  case sunray_fsm::SunrayState::RETURN:
    return "RETURN";
  case sunray_fsm::SunrayState::LAND:
    return "LAND";
  case sunray_fsm::SunrayState::EMERGENCY_LAND:
    return "EMERGENCY_LAND";
  case sunray_fsm::SunrayState::POSITION_CONTROL:
    return "POSITION_CONTROL";
  case sunray_fsm::SunrayState::VELOCITY_CONTROL:
    return "VELOCITY_CONTROL";
  case sunray_fsm::SunrayState::ATTITUDE_CONTROL:
    return "ATTITUDE_CONTROL";
  case sunray_fsm::SunrayState::COMPLEX_CONTROL:
    return "COMPLEX_CONTROL";
  case sunray_fsm::SunrayState::TRAJECTORY_CONTROL:
    return "TRAJECTORY_CONTROL";
  default:
    return "UNKNOWN_STATE";
  }
}

void fill_return_home_request(uav_control::ReturnHome::Request *req,
                              bool use_takeoff_homepoint,
                              const Eigen::Vector3d &target_position,
                              bool yaw_ctrl, double yaw,
                              double land_max_velocity) {
  if (!req) {
    return;
  }

  req->use_takeoff_homepoint = use_takeoff_homepoint;
  req->target_position.x = target_position.x();
  req->target_position.y = target_position.y();
  req->target_position.z = target_position.z();
  req->yaw_ctrl = yaw_ctrl;
  req->yaw = yaw;
  req->land_max_velocity = land_max_velocity;
}

void warn_if_return_land_type_ignored(int land_type) {
  if (land_type > 0) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: return land_type is no longer supported, "
                      "ignoring legacy value=%d",
                      land_type);
  }
}

void warn_if_yaw_rate_ignored(double target_yaw_rate) {
  if (std::abs(target_yaw_rate) > 1e-6) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: position yaw_rate is not supported, "
                      "ignoring value=%.3f",
                      target_yaw_rate);
  }
}

std::atomic<uint32_t> g_helper_control_sequence_id{1U};

uav_control::ControlMeta make_helper_control_meta() {
  uav_control::ControlMeta meta;
  meta.header.stamp = ros::Time::now();
  meta.source_id = uav_control::ControlMeta::SOURCE_API;
  meta.sequence_id =
      g_helper_control_sequence_id.fetch_add(1U, std::memory_order_relaxed);
  meta.timeout = ros::Duration(0.0);
  meta.allow_preempt = true;
  meta.replace_same_source = true;
  return meta;
}

double compute_trajectory_meta_timeout_s(const uav_control::Trajectory &trajectory,
                                         const ros::Time &meta_stamp) {
  if (trajectory.points.empty()) {
    return 0.0;
  }

  const ros::Time effective_start =
      trajectory.header.stamp.isZero() ? meta_stamp : trajectory.header.stamp;
  const double start_delay_s =
      std::max(0.0, (effective_start - meta_stamp).toSec());
  const double duration_s =
      std::max(0.0, trajectory.points.back().time_from_start.toSec());
  return start_delay_s + duration_s + kTrajectoryMetaTimeoutMarginS;
}

double sanitize_nominal_speed(double speed_mps) {
  return (speed_mps > 1e-6) ? speed_mps : 0.5;
}

double compute_segment_duration_s(const Eigen::Vector3d &from,
                                  const Eigen::Vector3d &to,
                                  double speed_mps) {
  const double safe_speed_mps = sanitize_nominal_speed(speed_mps);
  const double distance_m = (to - from).norm();
  if (distance_m <= 1e-6) {
    return kMinTrajectorySegmentTimeS;
  }
  return std::max(kMinTrajectorySegmentTimeS, distance_m / safe_speed_mps);
}

uav_control::Trajectory build_position_trajectory(
    const Eigen::Vector3d &start_position,
    const std::vector<Eigen::Vector3d> &position_list,
    double nominal_speed_mps) {
  uav_control::Trajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.reference_type = uav_control::Trajectory::FLAT_OUTPUT;
  trajectory.flat_output_order = uav_control::Trajectory::ACCELERATION;

  Eigen::Vector3d last_position = start_position;
  double cumulative_time_s = 0.0;
  for (const auto &position : position_list) {
    cumulative_time_s +=
        compute_segment_duration_s(last_position, position, nominal_speed_mps);

    uav_control::TrajectoryPointReference point_ref;
    point_ref.clear_all();
    point_ref.time_from_start = ros::Duration(cumulative_time_s);
    point_ref.set_position(position);
    trajectory.points.push_back(point_ref.toRosMessage());
    last_position = position;
  }
  return trajectory;
}

uav_control::Trajectory build_position_yaw_trajectory(
    const Eigen::Vector3d &start_position,
    const std::vector<std::pair<Eigen::Vector3d, double>> &point_list,
    double nominal_speed_mps, double yaw_rate_radps, bool enable_yaw_rate) {
  uav_control::Trajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.reference_type = uav_control::Trajectory::FLAT_OUTPUT;
  trajectory.flat_output_order = uav_control::Trajectory::ACCELERATION;

  Eigen::Vector3d last_position = start_position;
  double cumulative_time_s = 0.0;
  for (const auto &point : point_list) {
    cumulative_time_s += compute_segment_duration_s(last_position, point.first,
                                                    nominal_speed_mps);

    uav_control::TrajectoryPointReference point_ref;
    point_ref.clear_all();
    point_ref.time_from_start = ros::Duration(cumulative_time_s);
    point_ref.set_position(point.first);
    point_ref.set_yaw(point.second);
    if (enable_yaw_rate && std::abs(yaw_rate_radps) > 1e-6) {
      point_ref.set_yaw_rate(yaw_rate_radps);
    }
    trajectory.points.push_back(point_ref.toRosMessage());
    last_position = point.first;
  }
  return trajectory;
}

uav_control::Trajectory build_position_velocity_trajectory(
    const Eigen::Vector3d &start_position,
    const std::vector<std::pair<Eigen::Vector3d, double>> &point_list,
    double nominal_speed_mps) {
  uav_control::Trajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.reference_type = uav_control::Trajectory::FLAT_OUTPUT;
  trajectory.flat_output_order = uav_control::Trajectory::ACCELERATION;

  Eigen::Vector3d last_position = start_position;
  double cumulative_time_s = 0.0;
  for (const auto &point : point_list) {
    const double segment_speed_mps =
        (point.second > 1e-6) ? point.second : nominal_speed_mps;
    cumulative_time_s += compute_segment_duration_s(last_position, point.first,
                                                    segment_speed_mps);

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    const Eigen::Vector3d delta = point.first - last_position;
    const double distance = delta.norm();
    if (distance > 1e-6) {
      velocity = delta / distance * sanitize_nominal_speed(segment_speed_mps);
    }

    uav_control::TrajectoryPointReference point_ref;
    point_ref.clear_all();
    point_ref.time_from_start = ros::Duration(cumulative_time_s);
    point_ref.set_position(point.first);
    point_ref.set_velocity(velocity);
    trajectory.points.push_back(point_ref.toRosMessage());
    last_position = point.first;
  }
  return trajectory;
}

void warn_helper_api_unavailable(const char *api_name, const char *reason) {
  ROS_WARN_THROTTLE(1.0, "Sunray_Helper: %s is unavailable: %s", api_name,
                    reason);
}
} // namespace

Sunray_Helper::Sunray_Helper(ros::NodeHandle &nh) : nh_(nh) {
  uav_ns_ = resolve_uav_namespace(nh_);
  const std::unique_ptr<ros::NodeHandle> helper_cfg_nh =
      uav_ns_.empty()
          ? nullptr
          : std::unique_ptr<ros::NodeHandle>(
                new ros::NodeHandle("/" + uav_ns_ + "/sunray_helper"));
  const std::unique_ptr<ros::NodeHandle> uav_cfg_nh =
      uav_ns_.empty()
          ? nullptr
          : std::unique_ptr<ros::NodeHandle>(
                new ros::NodeHandle("/" + uav_ns_));

  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "takeoff_wait_s", &takeoff_wait_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "takeoff_block_timeout_s", &takeoff_wait_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "position_reached_tolerance_m",
                    &position_reached_tolerance_m_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(), "move_wait_s",
                    &block_wait_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "block_wait_timeout_s", &block_wait_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(), "land_wait_s",
                    &land_wait_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "land_block_timeout_s", &land_wait_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "land_confirm_wait_s", &land_confirm_timeout_s_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(), "wait_poll_hz",
                    &wait_poll_hz_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "yaw_reached_tolerance_rad", &yaw_reached_tolerance_rad_);
  load_helper_param(nh_, helper_cfg_nh.get(), uav_cfg_nh.get(),
                    "trajectory_nominal_speed_mps",
                    &trajectory_nominal_speed_mps_);

  const std::string ctrl_ns =
      uav_ns_.empty() ? "/sunray_control" : ("/" + uav_ns_ + "/sunray_control");
  ctrl_nh_ = ros::NodeHandle(ctrl_ns);

  velocity_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::VelocityCmdEnvelope>(
          "velocity_cmd_envelope", 10);
  attitude_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::AttitudeCmdEnvelope>(
          "attitude_cmd_envelope", 10);
  trajectory_envelope_pub_ =
      ctrl_nh_.advertise<uav_control::TrajectoryEnvelope>(
          "trajectory_envelope", 10);
  complex_cmd_pub_ =
      ctrl_nh_.advertise<uav_control::ComplexCmdEnvelope>(
          "complex_cmd_envelope", 10);
  fsm_state_sub_ =
      ctrl_nh_.subscribe("fsm_state", 10, &Sunray_Helper::fsm_state_cb, this);

  takeoff_client_ =
      ctrl_nh_.serviceClient<uav_control::Takeoff>("takeoff_request");
  land_client_ = ctrl_nh_.serviceClient<uav_control::Land>("land_request");
  return_client_ =
      ctrl_nh_.serviceClient<uav_control::ReturnHome>("return_request");
  position_cmd_client_ =
      ctrl_nh_.serviceClient<uav_control::PositionRequest>("position_request");

  try {
    ros::NodeHandle reader_nh =
        uav_ns_.empty() ? nh_ : ros::NodeHandle("/" + uav_ns_);
    px4_data_reader_.reset(new PX4_DataReader(reader_nh));
    px4_reader_ready_ = true;
  } catch (const std::exception &e) {
    ROS_WARN("Sunray_Helper: PX4_DataReader init failed: %s", e.what());
    px4_reader_ready_ = false;
  }
}

void Sunray_Helper::set_cached_fsm_state(sunray_fsm::SunrayState state) {
  std::lock_guard<std::mutex> lock(fsm_state_mutex_);
  fsm_state_ = state;
  fsm_state_received_ = true;
}

void Sunray_Helper::fsm_state_cb(const std_msgs::UInt8::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(fsm_state_mutex_);
  fsm_state_ = static_cast<sunray_fsm::SunrayState>(msg->data);
  fsm_state_received_ = true;
}

bool Sunray_Helper::wait_for_fsm_state(sunray_fsm::SunrayState expected_state,
                                       double timeout_s) {
  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : block_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    ros::spinOnce();
    {
      std::lock_guard<std::mutex> lock(fsm_state_mutex_);
      if (fsm_state_received_ && fsm_state_ == expected_state) {
        return true;
      }
    }
    rate.sleep();
  }

  sunray_fsm::SunrayState current_state = sunray_fsm::SunrayState::OFF;
  bool has_state = false;
  {
    std::lock_guard<std::mutex> lock(fsm_state_mutex_);
    current_state = fsm_state_;
    has_state = fsm_state_received_;
  }

  ROS_WARN("Sunray_Helper: wait for FSM state timeout, expected=%s current=%s "
           "received=%d timeout=%.3f",
           fsm_state_to_string(expected_state), fsm_state_to_string(current_state),
           static_cast<int>(has_state), effective_timeout_s);
  return false;
}

bool Sunray_Helper::wait_for_position_reached(
    const Eigen::Vector3d &target_position, double timeout_s) {
  if (!px4_reader_ready_ || !px4_data_reader_) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: PX4 reader unavailable, skip position "
                      "completion wait");
    return true;
  }

  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : block_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    ros::spinOnce();
    const Eigen::Vector3d current_position = get_uav_position();
    if ((current_position - target_position).norm() <=
        position_reached_tolerance_m_) {
      ROS_INFO(
          "[Sunray_Helper] PX4 position reached: current=(%.3f, %.3f, %.3f) "
          "target=(%.3f, %.3f, %.3f) tol=%.3f",
          current_position.x(), current_position.y(), current_position.z(),
          target_position.x(), target_position.y(), target_position.z(),
          position_reached_tolerance_m_);
      return true;
    }
    rate.sleep();
  }

  const Eigen::Vector3d current_position = get_uav_position();
  ROS_WARN(
      "Sunray_Helper: wait for position timeout, current=(%.3f, %.3f, %.3f) "
      "target=(%.3f, %.3f, %.3f) tol=%.3f timeout=%.3f",
      current_position.x(), current_position.y(), current_position.z(),
      target_position.x(), target_position.y(), target_position.z(),
      position_reached_tolerance_m_, effective_timeout_s);
  return false;
}

bool Sunray_Helper::wait_for_position_control_completed(double timeout_s) {
  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : block_wait_timeout_s_;
  if (!wait_for_fsm_state(sunray_fsm::SunrayState::HOVER, effective_timeout_s)) {
    sunray_fsm::SunrayState current_state = sunray_fsm::SunrayState::OFF;
    {
      std::lock_guard<std::mutex> lock(fsm_state_mutex_);
      current_state = fsm_state_;
    }
    ROS_WARN("Sunray_Helper: wait for position control completion timeout, "
             "expected FSM state=HOVER current=%s timeout=%.3f",
             fsm_state_to_string(current_state), effective_timeout_s);
    return false;
  }
  return true;
}

bool Sunray_Helper::wait_for_landed(double timeout_s) {
  if (!px4_reader_ready_ || !px4_data_reader_) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: PX4 reader unavailable, skip landing "
                      "completion wait");
    return true;
  }

  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : land_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    ros::spinOnce();
    const px4_data_types::SystemState system_state =
        px4_data_reader_->get_system_state();
    if (system_state.landed_state == px4_data_types::LandedState::kOnGround ||
        !system_state.armed) {
      const Eigen::Vector3d current_position = get_uav_position();
      ROS_INFO(
          "[Sunray_Helper] PX4 landed detected: current=(%.3f, %.3f, %.3f) "
          "armed=%d landed_state=%d",
          current_position.x(), current_position.y(), current_position.z(),
          static_cast<int>(system_state.armed),
          static_cast<int>(system_state.landed_state));
      return true;
    }
    rate.sleep();
  }

  const px4_data_types::SystemState system_state =
      px4_data_reader_->get_system_state();
  const Eigen::Vector3d current_position = get_uav_position();
  ROS_WARN(
      "Sunray_Helper: wait for landing timeout, armed=%d landed_state=%d "
      "current=(%.3f, %.3f, %.3f) timeout=%.3f",
      static_cast<int>(system_state.armed),
      static_cast<int>(system_state.landed_state), current_position.x(),
      current_position.y(), current_position.z(), effective_timeout_s);
  return false;
}

void Sunray_Helper::confirm_landed_best_effort(double timeout_s,
                                               const char *context) {
  if (wait_for_landed(timeout_s)) {
    return;
  }

  const Eigen::Vector3d current_position = get_uav_position();
  const Eigen::Vector3d current_velocity = get_uav_velocity_linear();
  const sunray_fsm::SunrayState fsm_state = get_statemachine_state();
  ROS_WARN(
      "Sunray_Helper: %s completed by FSM state, but PX4 landing confirmation "
      "did not arrive in time; fsm=%s current=(%.3f, %.3f, %.3f) "
      "vel=(%.3f, %.3f, %.3f)",
      context ? context : "landing",
      fsm_state_to_string(fsm_state), current_position.x(), current_position.y(),
      current_position.z(), current_velocity.x(), current_velocity.y(),
      current_velocity.z());
}

bool Sunray_Helper::wait_for_yaw_reached(double target_yaw, double timeout_s) {
  if (!px4_reader_ready_ || !px4_data_reader_) {
    ROS_WARN_THROTTLE(1.0,
                      "Sunray_Helper: PX4 reader unavailable, skip yaw "
                      "completion wait");
    return true;
  }

  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : block_wait_timeout_s_;
  ros::Rate rate(std::max(1.0, wait_poll_hz_));
  const ros::WallTime deadline =
      ros::WallTime::now() + ros::WallDuration(effective_timeout_s);
  const double wrapped_target_yaw = wrap_to_pi(target_yaw);

  while (ros::ok() && ros::WallTime::now() <= deadline) {
    ros::spinOnce();
    const double current_yaw = wrap_to_pi(get_uav_yaw_rad());
    if (angular_distance(current_yaw, wrapped_target_yaw) <=
        yaw_reached_tolerance_rad_) {
      ROS_INFO("[Sunray_Helper] yaw reached: current=%.3f target=%.3f tol=%.3f",
               current_yaw, wrapped_target_yaw, yaw_reached_tolerance_rad_);
      return true;
    }
    rate.sleep();
  }

  const double current_yaw = wrap_to_pi(get_uav_yaw_rad());
  ROS_WARN("Sunray_Helper: wait for yaw timeout, current=%.3f target=%.3f "
           "tol=%.3f timeout=%.3f",
           current_yaw, wrapped_target_yaw, yaw_reached_tolerance_rad_,
           effective_timeout_s);
  return false;
}

bool Sunray_Helper::wait_for_return_completed(double timeout_s) {
  const double effective_timeout_s =
      (timeout_s > 0.0) ? timeout_s : (block_wait_timeout_s_ + land_wait_timeout_s_);
  if (!wait_for_fsm_state(sunray_fsm::SunrayState::OFF, effective_timeout_s)) {
    return false;
  }
  confirm_landed_best_effort(land_confirm_timeout_s_, "return");
  return true;
}

bool Sunray_Helper::takeoff_async() {
  return takeoff_async(0.0, 0.0);
}

bool Sunray_Helper::takeoff_block() {
  return takeoff_block(0.0, 0.0);
}

bool Sunray_Helper::takeoff_async(double relative_takeoff_height,
                                  double max_takeoff_velocity) {
  if (!takeoff_client_.exists() &&
      !takeoff_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::Takeoff srv;
  srv.request.takeoff_relative_height = relative_takeoff_height;
  srv.request.takeoff_max_velocity = max_takeoff_velocity;
  if (!takeoff_client_.call(srv) || !srv.response.accepted) {
    return false;
  }
  set_cached_fsm_state(sunray_fsm::SunrayState::TAKEOFF);
  return true;
}

bool Sunray_Helper::takeoff_block(double relative_takeoff_height,
                                  double max_takeoff_velocity) {
  if (!takeoff_client_.exists() &&
      !takeoff_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::Takeoff srv;
  srv.request.takeoff_relative_height = relative_takeoff_height;
  srv.request.takeoff_max_velocity = max_takeoff_velocity;
  if (!takeoff_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    set_cached_fsm_state(sunray_fsm::SunrayState::TAKEOFF);
    return wait_for_fsm_state(sunray_fsm::SunrayState::HOVER,
                              takeoff_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::land_async() { return land_async(0, 0.0); }

bool Sunray_Helper::land_block() { return land_block(0, 0.0); }

bool Sunray_Helper::land_async(int land_type, double land_max_velocity) {
  if (!land_client_.exists() &&
      !land_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::Land srv;
  srv.request.land_type = land_type;
  srv.request.land_max_velocity = land_max_velocity;
  if (!land_client_.call(srv) || !srv.response.accepted) {
    return false;
  }
  set_cached_fsm_state(sunray_fsm::SunrayState::LAND);
  return true;
}

bool Sunray_Helper::land_block(int land_type, double land_max_velocity) {
  if (!land_client_.exists() &&
      !land_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::Land srv;
  srv.request.land_type = land_type;
  srv.request.land_max_velocity = land_max_velocity;
  if (!land_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    set_cached_fsm_state(sunray_fsm::SunrayState::LAND);
    if (!wait_for_fsm_state(sunray_fsm::SunrayState::OFF,
                            land_wait_timeout_s_)) {
      return false;
    }
    confirm_landed_best_effort(land_confirm_timeout_s_, "land");
    return true;
  }
  return false;
}

bool Sunray_Helper::return_async() {
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  fill_return_home_request(&srv.request, true, Eigen::Vector3d::Zero(), false,
                           0.0, 0.0);
  if (!return_client_.call(srv) || !srv.response.accepted) {
    return false;
  }
  set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  return true;
}

bool Sunray_Helper::return_block() {
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  fill_return_home_request(&srv.request, true, Eigen::Vector3d::Zero(), false,
                           0.0, 0.0);
  if (!return_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
    return wait_for_return_completed(block_wait_timeout_s_ + land_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::return_async(Eigen::Vector3d target_position) {
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  fill_return_home_request(&srv.request, false, target_position, false, 0.0,
                           0.0);
  if (!return_client_.call(srv) || !srv.response.accepted) {
    return false;
  }

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = target_position;
  set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  return true;
}

bool Sunray_Helper::return_block(Eigen::Vector3d target_position) {
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  fill_return_home_request(&srv.request, false, target_position, false, 0.0,
                           0.0);
  if (!return_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = target_position;
    set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
    return wait_for_return_completed(block_wait_timeout_s_ + land_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::return_async(Eigen::Vector3d target_position,
                                 double target_yaw) {
  return return_async(target_position, target_yaw, 0);
}

bool Sunray_Helper::return_block(Eigen::Vector3d target_position,
                                 double target_yaw) {
  return return_block(target_position, target_yaw, 0);
}

bool Sunray_Helper::return_async(Eigen::Vector3d target_position,
                                 double target_yaw, int land_type) {
  return return_async(target_position, target_yaw, land_type, 0.0);
}

bool Sunray_Helper::return_block(Eigen::Vector3d target_position,
                                 double target_yaw, int land_type) {
  return return_block(target_position, target_yaw, land_type, 0.0);
}

bool Sunray_Helper::return_async(Eigen::Vector3d target_position,
                                 double target_yaw, int land_type,
                                 double land_max_velocity) {
  warn_if_return_land_type_ignored(land_type);
  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  fill_return_home_request(&srv.request, false, target_position, true,
                           target_yaw, land_max_velocity);
  if (!return_client_.call(srv) || !srv.response.accepted) {
    return false;
  }

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = target_position;
  uav_target_.orientation = quat_from_yaw(target_yaw);
  set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
  return true;
}

bool Sunray_Helper::return_block(Eigen::Vector3d target_position,
                                 double target_yaw, int land_type,
                                 double land_max_velocity) {
  warn_if_return_land_type_ignored(land_type);

  if (!return_client_.exists() &&
      !return_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::ReturnHome srv;
  fill_return_home_request(&srv.request, false, target_position, true,
                           target_yaw, land_max_velocity);
  if (!return_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = target_position;
    uav_target_.orientation = quat_from_yaw(target_yaw);
    set_cached_fsm_state(sunray_fsm::SunrayState::RETURN);
    return wait_for_return_completed(block_wait_timeout_s_ + land_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::set_position_async(Eigen::Vector3d position_) {
  if (!position_cmd_client_.exists() &&
      !position_cmd_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::PositionRequest srv;
  srv.request.target_position.x = position_.x();
  srv.request.target_position.y = position_.y();
  srv.request.target_position.z = position_.z();
  srv.request.yaw_ctrl = false;
  srv.request.yaw = 0.0;
  if (!position_cmd_client_.call(srv) || !srv.response.accepted) {
    return false;
  }

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = position_;
  set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_block(Eigen::Vector3d position_) {
  if (!position_cmd_client_.exists() &&
      !position_cmd_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::PositionRequest srv;
  srv.request.target_position.x = position_.x();
  srv.request.target_position.y = position_.y();
  srv.request.target_position.z = position_.z();
  srv.request.yaw_ctrl = false;
  srv.request.yaw = 0.0;
  if (!position_cmd_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = position_;
    set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
    return wait_for_position_control_completed(block_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::set_position_list_async(
    std::vector<Eigen::Vector3d> position_list_) {
  if (position_list_.empty()) {
    return false;
  }
  if (position_list_.size() == 1U) {
    return set_position_async(position_list_.front());
  }
  return set_trajectory_async(build_position_trajectory(
      get_uav_position(), position_list_, trajectory_nominal_speed_mps_));
}

bool Sunray_Helper::set_position_list_block(
    std::vector<Eigen::Vector3d> position_list_) {
  if (position_list_.empty()) {
    return false;
  }
  for (const auto &target_position : position_list_) {
    if (!set_position_block(target_position)) {
      return false;
    }
  }
  return true;
}

bool Sunray_Helper::set_position_async(Eigen::Vector3d position_,
                                       double target_yaw) {
  if (!position_cmd_client_.exists() &&
      !position_cmd_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::PositionRequest srv;
  srv.request.target_position.x = position_.x();
  srv.request.target_position.y = position_.y();
  srv.request.target_position.z = position_.z();
  srv.request.yaw_ctrl = true;
  srv.request.yaw = target_yaw;
  if (!position_cmd_client_.call(srv) || !srv.response.accepted) {
    return false;
  }

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = position_;
  uav_target_.orientation = quat_from_yaw(target_yaw);
  set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_block(Eigen::Vector3d position_,
                                       double target_yaw) {
  if (!position_cmd_client_.exists() &&
      !position_cmd_client_.waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  uav_control::PositionRequest srv;
  srv.request.target_position.x = position_.x();
  srv.request.target_position.y = position_.y();
  srv.request.target_position.z = position_.z();
  srv.request.yaw_ctrl = true;
  srv.request.yaw = target_yaw;
  if (!position_cmd_client_.call(srv)) {
    return false;
  }
  if (srv.response.accepted) {
    uav_target_.timestamp = ros::Time::now();
    uav_target_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_target_.position = position_;
    uav_target_.orientation = quat_from_yaw(target_yaw);
    set_cached_fsm_state(sunray_fsm::SunrayState::POSITION_CONTROL);
    return wait_for_position_control_completed(block_wait_timeout_s_);
  }
  return false;
}

bool Sunray_Helper::set_position_async(Eigen::Vector3d target_position,
                                       double target_yaw,
                                       double target_yaw_rate) {
  warn_if_yaw_rate_ignored(target_yaw_rate);
  return set_position_async(target_position, target_yaw);
}

bool Sunray_Helper::set_position_block(Eigen::Vector3d target_position,
                                       double target_yaw,
                                       double target_yaw_rate) {
  warn_if_yaw_rate_ignored(target_yaw_rate);
  return set_position_block(target_position, target_yaw);
}

bool Sunray_Helper::set_position_list_async(
    std::vector<std::pair<Eigen::Vector3d, double>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  if (point_list_.size() == 1U) {
    return set_position_async(point_list_.front().first, point_list_.front().second);
  }
  return set_trajectory_async(build_position_yaw_trajectory(
      get_uav_position(), point_list_, trajectory_nominal_speed_mps_, 0.0,
      false));
}

bool Sunray_Helper::set_position_list_block(
    std::vector<std::pair<Eigen::Vector3d, double>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  for (const auto &target_point : point_list_) {
    if (!set_position_block(target_point.first, target_point.second)) {
      return false;
    }
  }
  return true;
}

bool Sunray_Helper::set_position_list_async(
    std::vector<std::pair<Eigen::Vector3d, double>> target_position_list,
    double target_yaw_rate) {
  if (target_position_list.empty()) {
    return false;
  }
  if (target_position_list.size() == 1U) {
    return set_position_async(target_position_list.front().first,
                              target_position_list.front().second,
                              target_yaw_rate);
  }
  return set_trajectory_async(build_position_yaw_trajectory(
      get_uav_position(), target_position_list, trajectory_nominal_speed_mps_,
      target_yaw_rate, true));
}

bool Sunray_Helper::set_position_list_block(
    std::vector<std::pair<Eigen::Vector3d, double>> target_position_list,
    double target_yaw_rate) {
  if (target_position_list.empty()) {
    return false;
  }
  warn_if_yaw_rate_ignored(target_yaw_rate);
  for (const auto &target_point : target_position_list) {
    if (!set_position_block(target_point.first, target_point.second)) {
      return false;
    }
  }
  return true;
}

bool Sunray_Helper::set_linear_velocity_async(Eigen::Vector3d velocity_) {
  uav_control::VelocityCmdEnvelope envelope;
  envelope.meta = make_helper_control_meta();
  envelope.payload.position_ctrl = false;
  envelope.payload.target_linear_velocity.x = velocity_.x();
  envelope.payload.target_linear_velocity.y = velocity_.y();
  envelope.payload.target_linear_velocity.z = velocity_.z();
  envelope.payload.target_angular_velocity.x = 0.0;
  envelope.payload.target_angular_velocity.y = 0.0;
  envelope.payload.target_angular_velocity.z = 0.0;
  velocity_cmd_pub_.publish(envelope);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.velocity = velocity_;
  set_cached_fsm_state(sunray_fsm::SunrayState::VELOCITY_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_velocity_async(
    Eigen::Vector3d position_, double velocity_) {
  Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();
  if (velocity_ > 0.0) {
    Eigen::Vector3d current = get_uav_position();
    Eigen::Vector3d diff = position_ - current;
    const double norm = diff.norm();
    if (norm > 1e-6) {
      cmd_vel = diff / norm * velocity_;
    }
  }
  uav_control::VelocityCmdEnvelope envelope;
  envelope.meta = make_helper_control_meta();
  envelope.payload.position_ctrl = true;
  envelope.payload.target_position.x = position_.x();
  envelope.payload.target_position.y = position_.y();
  envelope.payload.target_position.z = position_.z();
  envelope.payload.target_linear_velocity.x = cmd_vel.x();
  envelope.payload.target_linear_velocity.y = cmd_vel.y();
  envelope.payload.target_linear_velocity.z = cmd_vel.z();
  envelope.payload.target_angular_velocity.x = 0.0;
  envelope.payload.target_angular_velocity.y = 0.0;
  envelope.payload.target_angular_velocity.z = 0.0;
  velocity_cmd_pub_.publish(envelope);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = position_;
  uav_target_.velocity = cmd_vel;
  set_cached_fsm_state(sunray_fsm::SunrayState::VELOCITY_CONTROL);
  return true;
}

bool Sunray_Helper::set_position_velocity_block(
    Eigen::Vector3d position_, double velocity_) {
  if (!set_position_velocity_async(position_, velocity_)) {
    return false;
  }
  return wait_for_position_reached(position_, block_wait_timeout_s_);
}

bool Sunray_Helper::set_position_velocity_list_async(
    std::vector<std::pair<Eigen::Vector3d, double>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  if (point_list_.size() == 1U) {
    return set_position_velocity_async(point_list_.front().first,
                                       point_list_.front().second);
  }
  return set_trajectory_async(build_position_velocity_trajectory(
      get_uav_position(), point_list_, trajectory_nominal_speed_mps_));
}

bool Sunray_Helper::set_position_velocity_list_block(
    std::vector<std::pair<Eigen::Vector3d, double>> point_list_) {
  if (point_list_.empty()) {
    return false;
  }
  for (const auto &target_point : point_list_) {
    if (!set_position_velocity_block(target_point.first, target_point.second)) {
      return false;
    }
  }
  return true;
}

bool Sunray_Helper::set_yaw_async(double target_yaw) {
  uav_control::AttitudeCmdEnvelope envelope;
  envelope.meta = make_helper_control_meta();
  envelope.payload.yaw_ctrl_types = true;
  envelope.payload.yaw = target_yaw;
  attitude_cmd_pub_.publish(envelope);

  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.orientation = quat_from_yaw(target_yaw);
  set_cached_fsm_state(sunray_fsm::SunrayState::ATTITUDE_CONTROL);
  return true;
}

bool Sunray_Helper::set_yaw_block(double target_yaw) {
  if (!set_yaw_async(target_yaw)) {
    return false;
  }
  return wait_for_yaw_reached(target_yaw, block_wait_timeout_s_);
}

bool Sunray_Helper::set_yaw_adjust_async(double adjust_yaw) {
  uav_control::AttitudeCmdEnvelope envelope;
  envelope.meta = make_helper_control_meta();
  envelope.payload.yaw_ctrl_types = false;
  envelope.payload.yaw = adjust_yaw;
  attitude_cmd_pub_.publish(envelope);

  const double yaw = get_uav_yaw_rad() + adjust_yaw;
  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.orientation = quat_from_yaw(yaw);
  set_cached_fsm_state(sunray_fsm::SunrayState::ATTITUDE_CONTROL);
  return true;
}

bool Sunray_Helper::set_yaw_adjust_block(double adjust_yaw) {
  const double target_yaw = wrap_to_pi(get_uav_yaw_rad() + adjust_yaw);
  if (!set_yaw_adjust_async(adjust_yaw)) {
    return false;
  }
  return wait_for_yaw_reached(target_yaw, block_wait_timeout_s_);
}

bool Sunray_Helper::set_velocity_area(Eigen::Vector3d protect_area) {
  (void)protect_area;
  warn_helper_api_unavailable(
      "set_velocity_area",
      "velocity protect-area logic is not wired in controller/FSM");
  return false;
}

bool Sunray_Helper::clear_velocity_area(void) {
  warn_helper_api_unavailable(
      "clear_velocity_area",
      "velocity protect-area logic is not wired in controller/FSM");
  return false;
}

bool Sunray_Helper::set_anglar_velocity_async(Eigen::Vector3d target_velocity) {
  (void)target_velocity;
  warn_helper_api_unavailable(
      "set_anglar_velocity_async",
      "full angular-velocity control path is not implemented");
  return false;
}

bool Sunray_Helper::set_attitude_thrust_async(Eigen::Vector3d attitude,
                                              double turust) {
  (void)attitude;
  (void)turust;
  warn_helper_api_unavailable(
      "set_attitude_thrust_async",
      "external attitude/thrust takeover path is not implemented");
  return false;
}

bool Sunray_Helper::set_bodyrate_thrust_async(Eigen::Vector3d bodyrate,
                                              double thrust) {
  (void)bodyrate;
  (void)thrust;
  warn_helper_api_unavailable(
      "set_bodyrate_thrust_async",
      "external bodyrate/thrust takeover path is not implemented");
  return false;
}

bool Sunray_Helper::is_external_attitude_thrust_ready() {
  warn_helper_api_unavailable(
      "is_external_attitude_thrust_ready",
      "external attitude/thrust warmup and takeover logic is not implemented");
  return false;
}

bool Sunray_Helper::set_trajectory_point_async(
    const uav_control::TrajectoryPoint &target_trajpoint) {
  uav_control::TrajectoryEnvelope envelope;
  envelope.meta = make_helper_control_meta();
  envelope.payload.header.stamp = ros::Time::now();
  envelope.payload.reference_type = uav_control::Trajectory::FLAT_OUTPUT;
  envelope.payload.flat_output_order = uav_control::Trajectory::SNAP;
  envelope.payload.points.push_back(target_trajpoint);
  trajectory_envelope_pub_.publish(envelope);

  const uav_control::TrajectoryPointReference traj_ref(target_trajpoint);
  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = traj_ref.position;
  uav_target_.velocity = traj_ref.velocity;
  uav_target_.orientation = quat_from_yaw(traj_ref.heading);
  set_cached_fsm_state(sunray_fsm::SunrayState::TRAJECTORY_CONTROL);
  return true;
}

bool Sunray_Helper::set_trajectory_async(
    const uav_control::Trajectory &trajectory) {
  if (trajectory.points.empty()) {
    ROS_WARN_THROTTLE(1.0, "Sunray_Helper: empty trajectory is not allowed");
    return false;
  }
  uav_control::TrajectoryEnvelope envelope;
  envelope.meta = make_helper_control_meta();
  envelope.payload = trajectory;
  if (envelope.payload.header.stamp.isZero()) {
    envelope.payload.header.stamp = ros::Time::now();
  }
  envelope.meta.timeout = ros::Duration(
      compute_trajectory_meta_timeout_s(envelope.payload, envelope.meta.header.stamp));
  trajectory_envelope_pub_.publish(envelope);

  const uav_control::TrajectoryPointReference first_ref(trajectory.points.front());
  uav_target_.timestamp = ros::Time::now();
  uav_target_.coordinate_frame =
      uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
  uav_target_.position = first_ref.position;
  uav_target_.velocity = first_ref.velocity;
  uav_target_.orientation = quat_from_yaw(first_ref.heading);
  set_cached_fsm_state(sunray_fsm::SunrayState::TRAJECTORY_CONTROL);
  return true;
}

bool Sunray_Helper::set_complex_control() {
  warn_helper_api_unavailable(
      "set_complex_control",
      "FSM accepts complex envelope but no controller passthrough is wired");
  return false;
}

uav_control::UAVStateEstimate Sunray_Helper::get_uav_odometry() {
  if (px4_reader_ready_ && px4_data_reader_) {
    const auto pose = px4_data_reader_->get_local_pose();
    const auto vel = px4_data_reader_->get_local_velocity();
    const auto body_vel = px4_data_reader_->get_body_velocity();
    uav_odometry_.timestamp = ros::Time::now();
    uav_odometry_.coordinate_frame =
        uav_control::UAVStateEstimate::CoordinateFrame::LOCAL;
    uav_odometry_.position = pose.position;
    uav_odometry_.orientation = pose.orientation;
    uav_odometry_.velocity = vel.linear;
    uav_odometry_.bodyrates = body_vel.angular;
  }
  return uav_odometry_;
}

Eigen::Vector3d Sunray_Helper::get_uav_position() {
  return get_uav_odometry().position;
}

Eigen::Vector3d Sunray_Helper::get_uav_velocity_linear() {
  return get_uav_odometry().velocity;
}

Eigen::Vector3d Sunray_Helper::get_uav_velocity_angular() {
  return get_uav_odometry().bodyrates;
}

Eigen::Vector3d Sunray_Helper::get_uav_attitude_rpy_rad() {
  return rpy_from_quat(get_uav_odometry().orientation);
}

Eigen::Vector3d Sunray_Helper::get_uav_attitude_rpy_deg() {
  return get_uav_attitude_rpy_rad() * (180.0 / M_PI);
}

double Sunray_Helper::get_uav_yaw_rad() {
  return yaw_from_quat(get_uav_odometry().orientation);
}

double Sunray_Helper::get_uav_yaw_deg() {
  return get_uav_yaw_rad() * (180.0 / M_PI);
}

Eigen::Quaterniond Sunray_Helper::get_uav_attitude_quat() {
  return get_uav_odometry().orientation;
}

Eigen::Vector3d Sunray_Helper::get_target_position() {
  return uav_target_.position;
}

Eigen::Vector3d Sunray_Helper::get_target_velocity_linear() {
  return uav_target_.velocity;
}

Eigen::Vector3d Sunray_Helper::get_target_velocity_angular() {
  return uav_target_.bodyrates;
}

Eigen::Vector3d Sunray_Helper::get_target_attitude_rpy_rad() {
  return rpy_from_quat(uav_target_.orientation);
}

Eigen::Vector3d Sunray_Helper::get_target_attitude_rpy_deg() {
  return get_target_attitude_rpy_rad() * (180.0 / M_PI);
}

double Sunray_Helper::get_target_yaw_rad() {
  return yaw_from_quat(uav_target_.orientation);
}

double Sunray_Helper::get_target_yaw_deg() {
  return get_target_yaw_rad() * (180.0 / M_PI);
}

Eigen::Quaterniond Sunray_Helper::get_target_attitude_quat() {
  return uav_target_.orientation;
}

double Sunray_Helper::get_target_thrust() {
  return 0.0;
}

sunray_fsm::SunrayState Sunray_Helper::get_statemachine_state() {
  std::lock_guard<std::mutex> lock(fsm_state_mutex_);
  return fsm_state_;
}
