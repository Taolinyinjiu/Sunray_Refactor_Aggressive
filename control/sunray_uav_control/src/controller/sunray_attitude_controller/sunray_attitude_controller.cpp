#include "controller/sunray_attitude_controller/sunray_attitude_controller.hpp"

#include <algorithm>
#include <cmath>

#include "utils/curve/quintic_curve.hpp"

namespace uav_control {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp_symmetric(double value, double limit_abs) {
  const double limit = std::abs(limit_abs);
  return std::max(-limit, std::min(limit, value));
}

bool is_finite_quaternion(const Eigen::Quaterniond &q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z());
}

Eigen::Quaterniond normalized_or_identity(const Eigen::Quaterniond &q) {
  if (!is_finite_quaternion(q)) {
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Quaterniond normalized = q;
  if (normalized.norm() > 1e-9) {
    normalized.normalize();
    return normalized;
  }
  return Eigen::Quaterniond::Identity();
}

double yaw_from_quaternion(const Eigen::Quaterniond &q) {
  const Eigen::Quaterniond normalized = normalized_or_identity(q);
  const double siny_cosp =
      2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y());
  const double cosy_cosp =
      1.0 - 2.0 * (normalized.y() * normalized.y() +
                   normalized.z() * normalized.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Quaterniond quaternion_from_rpy(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

} // namespace

bool Attitude_Controller::load_param(ros::NodeHandle &nh) {
  nh.param("controller_update_frequency", ctrl_param_.controller_update_hz,
           ctrl_param_.controller_update_hz);
  nh.param("mass_kg", ctrl_param_.mass_kg, ctrl_param_.mass_kg);
  nh.param("gravity", ctrl_param_.gravity_mps2, ctrl_param_.gravity_mps2);

  double tilt_angle_max_deg = ctrl_param_.tilt_angle_max_rad * 180.0 / kPi;
  nh.param("tilt_angle_max", tilt_angle_max_deg, tilt_angle_max_deg);
  ctrl_param_.tilt_angle_max_rad = tilt_angle_max_deg * kPi / 180.0;

  double error_tol_x = error_tolerance_.x();
  double error_tol_y = error_tolerance_.y();
  double error_tol_z = error_tolerance_.z();
  nh.param("error_tolerance/pos_x", error_tol_x, error_tol_x);
  nh.param("error_tolerance/pos_y", error_tol_y, error_tol_y);
  nh.param("error_tolerance/pos_z", error_tol_z, error_tol_z);
  error_tolerance_ = Eigen::Vector3d(error_tol_x, error_tol_y, error_tol_z);

  double vel_max_x = velocity_max_.x();
  double vel_max_y = velocity_max_.y();
  double vel_max_z = velocity_max_.z();
  nh.param("max_velocity/x_vel", vel_max_x, vel_max_x);
  nh.param("max_velocity/y_vel", vel_max_y, vel_max_y);
  nh.param("max_velocity/z_vel", vel_max_z, vel_max_z);
  velocity_max_ = Eigen::Vector3d(vel_max_x, vel_max_y, vel_max_z);

  nh.param("sunray_attitude_controller/hover_percent",
           ctrl_param_.hover_percent, ctrl_param_.hover_percent);

  double pxy_int_max = ctrl_param_.int_max.x();
  double pz_int_max = ctrl_param_.int_max.z();
  nh.param("sunray_attitude_controller/pxy_int_max", pxy_int_max, pxy_int_max);
  nh.param("sunray_attitude_controller/pz_int_max", pz_int_max, pz_int_max);
  ctrl_param_.int_max = Eigen::Vector3d(pxy_int_max, pxy_int_max, pz_int_max);

  double kp_xy = ctrl_param_.Kp.x();
  double kp_z = ctrl_param_.Kp.z();
  double kv_xy = ctrl_param_.Kv.x();
  double kv_z = ctrl_param_.Kv.z();
  double kvi_xy = ctrl_param_.Kvi.x();
  double kvi_z = ctrl_param_.Kvi.z();
  nh.param("sunray_attitude_controller/Kp_xy", kp_xy, kp_xy);
  nh.param("sunray_attitude_controller/Kp_z", kp_z, kp_z);
  nh.param("sunray_attitude_controller/Kv_xy", kv_xy, kv_xy);
  nh.param("sunray_attitude_controller/Kv_z", kv_z, kv_z);
  nh.param("sunray_attitude_controller/Kvi_xy", kvi_xy, kvi_xy);
  nh.param("sunray_attitude_controller/Kvi_z", kvi_z, kvi_z);
  ctrl_param_.Kp = Eigen::Vector3d(kp_xy, kp_xy, kp_z);
  ctrl_param_.Kv = Eigen::Vector3d(kv_xy, kv_xy, kv_z);
  ctrl_param_.Kvi = Eigen::Vector3d(kvi_xy, kvi_xy, kvi_z);

  if (!(ctrl_param_.controller_update_hz > 0.0)) {
    ctrl_param_.controller_update_hz = 100.0;
  }
  if (!(ctrl_param_.mass_kg > 0.0)) {
    ctrl_param_.mass_kg = 0.96;
  }
  if (!(ctrl_param_.gravity_mps2 > 0.0)) {
    ctrl_param_.gravity_mps2 = 9.81;
  }
  if (!(ctrl_param_.hover_percent > 0.0)) {
    ctrl_param_.hover_percent = 0.37;
  }

  ROS_INFO(
      "[Attitude_Controller] params loaded: mass=%.3f gravity=%.3f "
      "hover_percent=%.3f tilt_max_deg=%.1f hz=%.1f",
      ctrl_param_.mass_kg, ctrl_param_.gravity_mps2, ctrl_param_.hover_percent,
      tilt_angle_max_deg, ctrl_param_.controller_update_hz);
  return true;
}

ControllerOutput Attitude_Controller::update(void) {
  if (!controller_ready_ && controller_state_ != ControllerState::UNDEFINED &&
      controller_state_ != ControllerState::OFF) {
    controller_state_ = ControllerState::EMERGENCY_LAND;

    ControllerOutput safe_output;
    safe_output.channel_enable(ControllerOutputMask::ATTITUDE);
    safe_output.channel_enable(ControllerOutputMask::THRUST);
    safe_output.attitude = normalized_or_identity(
        uav_current_state_.isValid() ? uav_current_state_.orientation
                                     : px4_attitude_);
    safe_output.thrust = ctrl_param_.min_command_thrust;
    return safe_output;
  }

  if (controller_state_ == ControllerState::UNDEFINED &&
      uav_current_state_.isValid()) {
    controller_state_ = ControllerState::OFF;
  }

  reset_takeoff_context_if_needed();
  reset_land_context_if_needed();

  switch (controller_state_) {
  case ControllerState::UNDEFINED:
    return handle_undefined_state();
  case ControllerState::OFF:
    return handle_off_state();
  case ControllerState::TAKEOFF:
    return handle_takeoff_state();
  case ControllerState::HOVER:
    return handle_hover_state();
  case ControllerState::MOVE:
    return handle_move_state();
  case ControllerState::LAND:
  case ControllerState::EMERGENCY_LAND:
    return handle_land_state();
  default:
    return ControllerOutput();
  }
}

void Attitude_Controller::reset_takeoff_context_if_needed() {
  if (controller_state_ != ControllerState::TAKEOFF && takeoff_initialized_) {
    takeoff_initialized_ = false;
    takeoff_start_time_ = ros::Time(0);
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
    takeoff_singlecurve_time_ = 0.0;
  }
}

void Attitude_Controller::reset_land_context_if_needed() {
  if (controller_state_ != ControllerState::LAND &&
      controller_state_ != ControllerState::EMERGENCY_LAND &&
      land_initialized_) {
    land_initialized_ = false;
    land_start_time_ = ros::Time(0);
    land_holdstart_time_ = ros::Time(0);
    land_holdkeep_time_ = ros::Time(0);
    land_singlecurve_time_ = 0.0;
    land_low_velocity_start_time_ = ros::Time(0);
    land_touchdown_detected_time_ = ros::Time(0);
    reset_integrator();
  }
}

void Attitude_Controller::reset_integrator() { int_e_v_.setZero(); }

Attitude_Controller::DesiredState Attitude_Controller::build_desired_state(
    const TrajectoryPointReference &trajectory_ref) const {
  DesiredState desired_state;
  desired_state.position =
      trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::POSITION)
          ? trajectory_ref.position
          : uav_current_state_.position;
  desired_state.velocity =
      trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::VELOCITY)
          ? trajectory_ref.velocity
          : Eigen::Vector3d::Zero();
  desired_state.acceleration =
      trajectory_ref.is_field_enabled(
          TrajectoryPointReference::Field::ACCELERATION)
          ? trajectory_ref.acceleration
          : Eigen::Vector3d::Zero();

  if (trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::YAW)) {
    desired_state.yaw = trajectory_ref.heading;
  } else if (trajectory_ref.is_field_enabled(
                 TrajectoryPointReference::Field::ORIENTATION)) {
    desired_state.yaw = yaw_from_quaternion(trajectory_ref.orientation);
  } else {
    desired_state.yaw = yaw_from_quaternion(uav_current_state_.orientation);
  }
  return desired_state;
}

ControllerOutput Attitude_Controller::solve_attitude_thrust(
    const TrajectoryPointReference &trajectory_ref, bool allow_xy_integral) {
  return solve_attitude_thrust(build_desired_state(trajectory_ref),
                               allow_xy_integral);
}

ControllerOutput Attitude_Controller::solve_attitude_thrust(
    const DesiredState &desired_state, bool allow_xy_integral) {
  ControllerOutput output;
  if (!uav_current_state_.isValid()) {
    return output;
  }

  Eigen::Vector3d pos_error = desired_state.position - uav_current_state_.position;
  Eigen::Vector3d vel_error = desired_state.velocity - uav_current_state_.velocity;

  for (int i = 0; i < 3; ++i) {
    if (std::abs(pos_error[i]) > ctrl_param_.max_position_error_m) {
      pos_error[i] = (pos_error[i] > 0.0) ? 1.0 : -1.0;
    }
    if (std::abs(vel_error[i]) > ctrl_param_.max_velocity_error_mps) {
      vel_error[i] = (vel_error[i] > 0.0) ? 2.0 : -2.0;
    }
  }

  const bool enable_integral =
      allow_xy_integral && controller_state_ != ControllerState::OFF &&
      controller_state_ != ControllerState::UNDEFINED;

  for (int i = 0; i < 2; ++i) {
    if (enable_integral &&
        std::abs(pos_error[i]) < ctrl_param_.position_integral_start_error_xy_m) {
      int_e_v_[i] += pos_error[i] / ctrl_param_.controller_update_hz;
      int_e_v_[i] = clamp_symmetric(int_e_v_[i], ctrl_param_.int_max[i]);
    } else {
      int_e_v_[i] = 0.0;
    }
  }

  if (enable_integral &&
      std::abs(pos_error[2]) < ctrl_param_.position_integral_start_error_z_m) {
    int_e_v_[2] += pos_error[2] / ctrl_param_.controller_update_hz;
    int_e_v_[2] = clamp_symmetric(int_e_v_[2], ctrl_param_.int_max[2]);
  } else {
    int_e_v_[2] = 0.0;
  }

  if (std::abs(desired_state.velocity.x()) > 1e-9 ||
      std::abs(desired_state.velocity.y()) > 1e-9 ||
      std::abs(desired_state.velocity.z()) > 1e-9) {
    int_e_v_[0] = 0.0;
    int_e_v_[1] = 0.0;
  }

  const Eigen::Vector3d des_acc =
      desired_state.acceleration +
      ctrl_param_.Kp.cwiseProduct(pos_error) +
      ctrl_param_.Kv.cwiseProduct(vel_error) +
      ctrl_param_.Kvi.cwiseProduct(int_e_v_);

  Eigen::Vector3d f_des = des_acc * ctrl_param_.mass_kg;
  f_des.z() += ctrl_param_.mass_kg * ctrl_param_.gravity_mps2;

  const double nominal_weight = ctrl_param_.mass_kg * ctrl_param_.gravity_mps2;
  if (f_des.z() < 0.5 * nominal_weight && std::abs(f_des.z()) > 1e-6) {
    f_des = f_des / f_des.z() * (0.5 * nominal_weight);
  } else if (f_des.z() > 2.0 * nominal_weight && std::abs(f_des.z()) > 1e-6) {
    f_des = f_des / f_des.z() * (2.0 * nominal_weight);
  }

  const double tilt_tan = std::tan(ctrl_param_.tilt_angle_max_rad);
  if (std::abs(f_des.x() / std::max(1e-6, std::abs(f_des.z()))) > tilt_tan) {
    f_des.x() = ((f_des.x() > 0.0) ? 1.0 : -1.0) * std::abs(f_des.z()) *
                tilt_tan;
  }
  if (std::abs(f_des.y() / std::max(1e-6, std::abs(f_des.z()))) > tilt_tan) {
    f_des.y() = ((f_des.y() > 0.0) ? 1.0 : -1.0) * std::abs(f_des.z()) *
                tilt_tan;
  }

  const double current_yaw = yaw_from_quaternion(uav_current_state_.orientation);
  const Eigen::Matrix3d yaw_rotation =
      Eigen::AngleAxisd(current_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d f_c = yaw_rotation.transpose() * f_des;

  const double roll = std::atan2(-f_c.y(), f_c.z());
  const double pitch = std::atan2(f_c.x(), f_c.z());
  const double yaw = desired_state.yaw;
  const Eigen::Quaterniond desired_attitude =
      quaternion_from_rpy(roll, pitch, yaw);

  const Eigen::Matrix3d wRb =
      normalized_or_identity(uav_current_state_.orientation).toRotationMatrix();
  const Eigen::Vector3d z_b_curr = wRb.col(2);
  const double u1 = f_des.dot(z_b_curr);
  const double full_thrust =
      nominal_weight / std::max(1e-6, ctrl_param_.hover_percent);

  output.channel_enable(ControllerOutputMask::ATTITUDE);
  output.channel_enable(ControllerOutputMask::THRUST);
  output.attitude = desired_attitude;
  output.thrust =
      std::max(ctrl_param_.min_command_thrust, std::min(1.0, u1 / full_thrust));
  return output;
}

ControllerOutput Attitude_Controller::handle_undefined_state() {
  return ControllerOutput();
}

ControllerOutput Attitude_Controller::handle_off_state() {
  reset_integrator();
  return ControllerOutput();
}

ControllerOutput Attitude_Controller::handle_takeoff_state() {
  if (!px4_arm_state_) {
    ControllerOutput output;
    output.channel_enable(ControllerOutputMask::ATTITUDE);
    output.channel_enable(ControllerOutputMask::THRUST);
    output.attitude = normalized_or_identity(
        uav_current_state_.isValid() ? uav_current_state_.orientation
                                     : px4_attitude_);
    output.thrust = ctrl_param_.min_command_thrust;
    return output;
  }

  if (!takeoff_initialized_) {
    takeoff_initialized_ = true;
    takeoff_start_time_ = ros::Time::now();
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
    takeoff_yaw_ = yaw_from_quaternion(uav_current_state_.orientation);
    reset_integrator();
  }

  constexpr uint8_t kTakeoffReadyMask = 0x07U;
  uint8_t takeoff_ready = 0U;
  const double ex =
      std::abs(takeoff_expect_position_.x() - uav_current_state_.position.x());
  const double ey =
      std::abs(takeoff_expect_position_.y() - uav_current_state_.position.y());
  const double ez =
      std::abs(takeoff_expect_position_.z() - uav_current_state_.position.z());
  if (ex < error_tolerance_.x()) {
    takeoff_ready |= (1U << 0);
  }
  if (ey < error_tolerance_.y()) {
    takeoff_ready |= (1U << 1);
  }
  if (ez < error_tolerance_.z()) {
    takeoff_ready |= (1U << 2);
  }

  const ros::Time now = ros::Time::now();
  const double hold_required =
      (takeoff_success_time_ > 2.0) ? takeoff_success_time_ : 2.0;
  if (takeoff_ready == kTakeoffReadyMask) {
    if (takeoff_holdstart_time_.isZero()) {
      takeoff_holdstart_time_ = now;
      takeoff_holdkeep_time_ = now;
    } else {
      takeoff_holdkeep_time_ = now;
      if ((now - takeoff_holdstart_time_).toSec() >= hold_required) {
        TrajectoryPointReference hold_ref;
        hold_ref.set_position(takeoff_expect_position_);
        hold_ref.set_yaw(takeoff_yaw_);
        trajectory_ = hold_ref.toRosMessage();
        controller_state_ = ControllerState::HOVER;
        return solve_attitude_thrust(hold_ref, true);
      }
    }
  } else {
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
  }

  TrajectoryPointReference desired_ref;
  const Eigen::Vector3d start_position = home_position_;
  const Eigen::Vector3d start_velocity = Eigen::Vector3d::Zero();
  const Eigen::Vector3d stop_position = takeoff_expect_position_;
  const Eigen::Vector3d stop_velocity = Eigen::Vector3d::Zero();

  if (takeoff_singlecurve_time_ == 0.0) {
    const auto min_duration_ret =
        curve::solve_quintic_min_duration_from_max_speed(
            start_position, stop_position, takeoff_max_velocity_);
    if (min_duration_ret.valid) {
      takeoff_singlecurve_time_ = min_duration_ret.min_duration_s * 2.0;
    }
  }

  if (takeoff_singlecurve_time_ > takeoff_singlecurve_limit_time_) {
    const auto curve_result = curve::evaluate_quintic_curve(
        start_position, start_velocity, stop_position, stop_velocity,
        takeoff_start_time_.toSec(), takeoff_singlecurve_time_, now.toSec());
    if (curve_result.valid) {
      desired_ref.set_position(curve_result.position);
      desired_ref.set_velocity(curve_result.velocity);
      desired_ref.set_acceleration(curve_result.acceleration);
    } else {
      desired_ref.set_position(takeoff_expect_position_);
    }
  } else {
    desired_ref.set_position(takeoff_expect_position_);
  }

  desired_ref.set_yaw(takeoff_yaw_);
  return solve_attitude_thrust(desired_ref, true);
}

ControllerOutput Attitude_Controller::handle_hover_state() {
  TrajectoryPointReference desired_ref(trajectory_);
  if (!desired_ref.is_field_enabled(TrajectoryPointReference::Field::POSITION)) {
    desired_ref.set_position(uav_current_state_.position);
  }
  if (!desired_ref.is_field_enabled(TrajectoryPointReference::Field::YAW)) {
    desired_ref.set_yaw(yaw_from_quaternion(uav_current_state_.orientation));
  }
  return solve_attitude_thrust(desired_ref, true);
}

ControllerOutput Attitude_Controller::handle_move_state() {
  (void)update_trajectory_reference_from_buffer(ros::Time::now());
  const TrajectoryPointReference trajectory_ref(trajectory_);
  return solve_attitude_thrust(trajectory_ref, true);
}

ControllerOutput Attitude_Controller::handle_land_state() {
  if (land_type_ == 1U) {
    return ControllerOutput();
  }

  if (!land_initialized_) {
    land_initialized_ = true;
    land_start_time_ = ros::Time::now();
    land_holdstart_time_ = ros::Time(0);
    land_holdkeep_time_ = ros::Time(0);
    land_low_velocity_start_time_ = ros::Time(0);
    land_touchdown_detected_time_ = ros::Time(0);
    land_expect_position_.x() = uav_current_state_.position.x();
    land_expect_position_.y() = uav_current_state_.position.y();
    land_yaw_ = yaw_from_quaternion(uav_current_state_.orientation);
    reset_integrator();
  }

  const ros::Time now = ros::Time::now();
  const bool near_ground =
      ground_reference_initialized_ &&
      (uav_current_state_.position.z() <=
       ground_reference_z_ + land_touchdown_height_threshold_m_);
  const bool velocity_low =
      std::abs(uav_current_state_.velocity.x()) <
          land_touchdown_velocity_threshold_mps_ &&
      std::abs(uav_current_state_.velocity.y()) <
          land_touchdown_velocity_threshold_mps_ &&
      std::abs(uav_current_state_.velocity.z()) <
          land_touchdown_velocity_threshold_mps_;

  if (near_ground && velocity_low) {
    if (land_low_velocity_start_time_.isZero()) {
      land_low_velocity_start_time_ = now;
    }
  } else {
    land_low_velocity_start_time_ = ros::Time(0);
  }

  const bool landed_by_velocity =
      near_ground && !land_low_velocity_start_time_.isZero() &&
      (now - land_low_velocity_start_time_).toSec() >=
          land_touchdown_velocity_hold_time_s_;
  const bool landed_detected = px4_land_status_ || landed_by_velocity;

  DesiredState desired_state;
  desired_state.position =
      Eigen::Vector3d(land_expect_position_.x(), land_expect_position_.y(),
                      uav_current_state_.position.z());
  desired_state.yaw = land_yaw_;

  if (landed_detected) {
    if (land_touchdown_detected_time_.isZero()) {
      land_touchdown_detected_time_ = now;
    }
    desired_state.velocity =
        Eigen::Vector3d(0.0, 0.0, -std::abs(land_touchdown_downpress_speed_mps_));
    if ((now - land_touchdown_detected_time_).toSec() >=
        land_touchdown_downpress_time_s_) {
      controller_state_ = ControllerState::OFF;
    }
    return solve_attitude_thrust(desired_state, false);
  }

  const double descent_speed =
      (land_max_velocity_ < 0.0) ? land_max_velocity_
                                 : -std::max(0.1, land_max_velocity_);
  desired_state.velocity = Eigen::Vector3d(0.0, 0.0, descent_speed);
  return solve_attitude_thrust(desired_state, true);
}

} // namespace uav_control
