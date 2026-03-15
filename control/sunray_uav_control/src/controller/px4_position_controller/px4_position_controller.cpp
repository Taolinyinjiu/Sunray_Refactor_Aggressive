#include "controller/px4_position_controller/px4_position_controller.h"
#include "utils/curve/quintic_curve.hpp"
#include <algorithm>
#include <cmath>

namespace uav_control {
namespace {

bool isFiniteQuaternion(const Eigen::Quaterniond &q) {
  return std::isfinite(q.w()) && std::isfinite(q.x()) &&
         std::isfinite(q.y()) && std::isfinite(q.z());
}

double yawFromQuaternion(const Eigen::Quaterniond &q) {
  Eigen::Quaterniond normalized = q;
  if (normalized.norm() > 1e-9) {
    normalized.normalize();
  } else {
    normalized = Eigen::Quaterniond::Identity();
  }

  const double siny_cosp =
      2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y());
  const double cosy_cosp =
      1.0 - 2.0 * (normalized.y() * normalized.y() +
                   normalized.z() * normalized.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

double clampSymmetric(double value, double limit_abs) {
  const double limit = std::abs(limit_abs);
  return std::max(-limit, std::min(limit, value));
}

} // namespace

// 根据控制器当前状态，进入对应的函数
ControllerOutput Position_Controller::update(void) {
  // 检查当前控制器是否稳定，并且需要满足控制器此时不在UNDEFINE或者OFF阶段
  if (!controller_ready_ && controller_state_ != ControllerState::UNDEFINED &&
      controller_state_ != ControllerState::OFF) {
    // 切换为紧急降落
    controller_state_ = ControllerState::EMERGENCY_LAND;
    // 本次环节主要用于稳定无人机姿态，切换为速度控制，输出为零速度
    ControllerOutput safe_output;
    safe_output.channel_enable(ControllerOutputMask::VELOCITY);
    safe_output.velocity = Eigen::Vector3d::Zero();
    return safe_output;
  }

  // 参数已加载且状态估计有效时，允许从 UNDEFINED 进入 OFF 待机态。
  if (controller_state_ == ControllerState::UNDEFINED &&
      uav_current_state_.isValid()) {
    controller_state_ = ControllerState::OFF;
  }
  // 重置起飞降落上下文
  reset_takeoff_context_if_needed();
  reset_land_context_if_needed();
  // 根据状态机状态进入对应的分支
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

void Position_Controller::reset_takeoff_context_if_needed() {
  // 如果当前状态不为TAKEOFF状态，但是TAKEOFF标识符指示初始化了，说明当前已经离开TAKEOFF阶段，需要重置起飞上下文参数
  if (controller_state_ != ControllerState::TAKEOFF && takeoff_initialized_) {
    // 重置起飞上下文标识
    takeoff_initialized_ = false;
    // 清空时间参数
    takeoff_start_time_ = ros::Time(0);
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
    takeoff_singlecurve_time_ = 0.0;
  }
}

void Position_Controller::reset_land_context_if_needed() {
  // 如果当前状态不为LAND状态，但是LAND标识符又指示初始化了，说明当前已经结束了LAND阶段，需要重置LAND参数
  if (controller_state_ != ControllerState::LAND &&
      controller_state_ != ControllerState::EMERGENCY_LAND &&
      land_initialized_) {
    // 重置降落上下文标识
    land_initialized_ = false;
    // 清空时间参数
    land_start_time_ = ros::Time(0);
    land_holdstart_time_ = ros::Time(0);
    land_holdkeep_time_ = ros::Time(0);
    land_singlecurve_time_ = 0.0;
    land_low_velocity_start_time_ = ros::Time(0);
    land_touchdown_detected_time_ = ros::Time(0);
  }
}

ControllerOutput Position_Controller::handle_undefined_state() {
  // 如果是UNDEFINE阶段，则说明无人机并没有做好准备，因此此时什么都不执行
  return ControllerOutput(); // 空构造函数，返回也是空结构体
}

ControllerOutput Position_Controller::handle_off_state() {
  // OFF阶段，此时无人机在地面上静止，不进行输出
  return ControllerOutput();
}

ControllerOutput Position_Controller::handle_takeoff_state() {
  ControllerOutput temp_output;
  /** ---------------起飞准备工作----------------- */

  // 未解锁：持续发零速度，保持控制链路活跃
  if (!px4_arm_state_) {
    temp_output.channel_enable(ControllerOutputMask::VELOCITY);
    temp_output.velocity.x() = 0.0;
    temp_output.velocity.y() = 0.0;
    temp_output.velocity.z() = 0.0;
    return temp_output;
  }
  // 检查起飞标识符，如果takeoff_initialized_=false,则表示当前为第一次进入takeoff阶段，需要重置参数
  if (!takeoff_initialized_) {
    // 设置为起飞参数已经初始化
    takeoff_initialized_ = true;
    // 设置起飞时间为当前
    takeoff_start_time_ = ros::Time::now();
    // 重置稳定判断时间戳
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
  }

  /** ---------------到位误差检测----------------- */
  constexpr uint8_t kTakeoffReadyMask = 0x07U; // 111的二进制是0x07
  uint8_t takeoff_ready = 0U;
  // 计算误差
  const double ex =
      std::abs(takeoff_expect_position_.x() - uav_current_state_.position.x());
  const double ey =
      std::abs(takeoff_expect_position_.y() - uav_current_state_.position.y());
  const double ez =
      std::abs(takeoff_expect_position_.z() - uav_current_state_.position.z());
  // 如果误差在容许的范围内，置位
  if (ex < error_tolerance_.x()) {
    takeoff_ready |= (1U << 0);
  }
  if (ey < error_tolerance_.y()) {
    takeoff_ready |= (1U << 1);
  }
  if (ez < error_tolerance_.z()) {
    takeoff_ready |= (1U << 2);
  }

  // 根据在期望的起飞位置误差内的持续时间来判断是否达到了稳定的阶段
  const ros::Time now = ros::Time::now();
  // 强制要求至少要2s用于判断是否稳定悬停
  const double hold_required =
      (takeoff_success_time_ > 2.0) ? takeoff_success_time_ : 2.0;
  // 如果三轴上的误差都满足要求，开始计算持续时间是否满足要求
  if (takeoff_ready == kTakeoffReadyMask) {
    // 如果保持的开始时间为0
    if (takeoff_holdstart_time_.isZero()) {
      takeoff_holdstart_time_ = now;
      takeoff_holdkeep_time_ = now;
    } else {
      takeoff_holdkeep_time_ = now;
      const ros::Duration hold_time = now - takeoff_holdstart_time_;
      if (hold_time.toSec() >= hold_required) {
        // 设置轨迹点为当前起飞参数
        TrajectoryPointReference hold_trajectory(trajectory_);
        hold_trajectory.set_position(takeoff_expect_position_);
        trajectory_ = hold_trajectory.toRosMessage();
        // 切换到HOVER状态
        controller_state_ = ControllerState::HOVER;
        // 构造零速输出
        temp_output.channel_enable(ControllerOutputMask::POSITION);
        temp_output.channel_enable(ControllerOutputMask::VELOCITY);
        temp_output.position = takeoff_expect_position_;
        temp_output.velocity.setZero();
        return temp_output;
      }
    }
  } else {
    // 任一轴超出容差，重置计时
    takeoff_holdstart_time_ = ros::Time(0);
    takeoff_holdkeep_time_ = ros::Time(0);
  }
  /** ---------------计算起飞阶段需要的时间----------------- */
  // 1. 起点位置和速度
  Eigen::Vector3d start_position = home_position_;
  Eigen::Vector3d start_velocity;
  start_velocity.setZero();
  // 2. 终点位置和速度
  Eigen::Vector3d stop_position = takeoff_expect_position_;
  Eigen::Vector3d stop_velocity;
  stop_velocity.setZero();
  // 3. 根据当前相对高度和最大速度评估起飞时间，如果小于0.5则进行调整
  if (takeoff_singlecurve_time_ == 0.0) {
    // 根据最大速度反推起飞时间
    double curve_max_velocity = takeoff_max_velocity_;
    const auto min_duration_ret =
        curve::solve_quintic_min_duration_from_max_speed(
            start_position, stop_position, curve_max_velocity);
    // 如果曲线计算正确，则valid为true
    if (min_duration_ret.valid == true) {
      // 设置理论时间
      takeoff_singlecurve_time_ = min_duration_ret.min_duration_s;
      // 考虑到实际中的一些原因，我们将这个值扩大一些
      takeoff_singlecurve_time_ *= 2;
    } else {
      // 如果曲线得不到有效的计算值，说明参数有问题，此时切换到LAND进行降落
      controller_state_ = ControllerState::LAND;
      temp_output.clear_all();
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.velocity.setZero();
      return temp_output;
    }
  }
  /** ---------------计算起飞五次项曲线参数----------------- */
  if (takeoff_singlecurve_time_ > takeoff_singlecurve_limit_time_) {

    // 如果曲线用时大于限制时间，则认为是符合要求的,根据曲线参数生成对应的指令
    const auto curve_result = curve::evaluate_quintic_curve(
        start_position, start_velocity, stop_position, stop_velocity,
        takeoff_start_time_.toSec(), takeoff_singlecurve_time_,
        ros::Time::now().toSec());
    if (curve_result.valid == true) {
      // 五次项曲线输出 位置 速度 加速度
      temp_output.channel_enable(ControllerOutputMask::POSITION);
      temp_output.channel_enable(ControllerOutputMask::VELOCITY);
      temp_output.channel_enable(ControllerOutputMask::ACCELERATION);
      temp_output.position = curve_result.position;
      temp_output.velocity = curve_result.velocity;
      temp_output.acceleration_or_force = curve_result.acceleration;
    } else if (curve_result.valid == false) {
      // 如果曲线生成有问题，那就直接给目标点吧
      temp_output.channel_enable(ControllerOutputMask::POSITION);
      temp_output.position = takeoff_expect_position_;
    }
  } else {
    // TODO：这里的逻辑需要修改
    // 如果曲线用时小于曲线限制时间,说明速度比较大，或者起飞高度比较小
    // 持续输出起飞目标点
    temp_output.channel_enable(ControllerOutputMask::POSITION);
    temp_output.position = takeoff_expect_position_;
  }
  /** ---------------输出控制指令---------------- */
  return temp_output;
}

ControllerOutput Position_Controller::handle_hover_state() {
  ControllerOutput temp_output;
  // HOVER状态下，设置输出为当前轨迹点
  const TrajectoryPointReference trajectory_ref(trajectory_);
  temp_output.channel_enable(ControllerOutputMask::POSITION);
  temp_output.position = trajectory_ref.position;
  if (trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::YAW)) {
    temp_output.channel_enable(ControllerOutputMask::YAW);
    temp_output.yaw = trajectory_ref.heading;
  }
  return temp_output;
}

ControllerOutput Position_Controller::handle_move_state() {
  ControllerOutput temp_output;
  (void)update_trajectory_reference_from_buffer(ros::Time::now());
  const TrajectoryPointReference trajectory_ref(trajectory_);
  // 当切换到MOVE时，通常是接受到了相关的控制指令
  if (trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::POSITION)) {
    temp_output.channel_enable(ControllerOutputMask::POSITION);
    temp_output.position = trajectory_ref.position;
  }
  if (trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::VELOCITY)) {
    temp_output.channel_enable(ControllerOutputMask::VELOCITY);
    temp_output.velocity = trajectory_ref.velocity;
  }
  if (trajectory_ref.is_field_enabled(
          TrajectoryPointReference::Field::ACCELERATION)) {
    temp_output.channel_enable(ControllerOutputMask::ACCELERATION);
    temp_output.acceleration_or_force = trajectory_ref.acceleration;
  }
  if (trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::YAW)) {
    temp_output.channel_enable(ControllerOutputMask::YAW);
    temp_output.yaw = trajectory_ref.heading;
  }
  if (trajectory_ref.is_field_enabled(TrajectoryPointReference::Field::YAW_RATE)) {
    temp_output.channel_enable(ControllerOutputMask::YAW_RATE);
    temp_output.yaw_rate = trajectory_ref.heading_rate;
  }

  // position_controller
  // 不支持对姿态，推力，加加速度，加加加速度进行调整
  return temp_output;
}

ControllerOutput Position_Controller::handle_land_state() {
  ControllerOutput temp_output;
  const ros::Time now = ros::Time::now();

  if (!land_initialized_) {
    land_initialized_ = true;
    land_start_time_ = now;
    land_holdstart_time_ = ros::Time(0);
    land_holdkeep_time_ = ros::Time(0);
    land_low_velocity_start_time_ = ros::Time(0);
    land_touchdown_detected_time_ = ros::Time(0);
    land_expect_position_.x() = uav_current_state_.position.x();
    land_expect_position_.y() = uav_current_state_.position.y();

    if (isFiniteQuaternion(uav_current_state_.orientation) &&
        uav_current_state_.orientation.norm() > 1e-9) {
      land_yaw_ = yawFromQuaternion(uav_current_state_.orientation);
    } else if (isFiniteQuaternion(px4_attitude_) &&
               px4_attitude_.norm() > 1e-9) {
      land_yaw_ = yawFromQuaternion(px4_attitude_);
    } else {
      land_yaw_ = 0.0;
    }

    ROS_INFO(
        "[Position_Controller] LAND init: hold_xy=(%.3f, %.3f) current_z=%.3f "
        "ground_z=%.3f land_type=%u yaw=%.3f",
        land_expect_position_.x(), land_expect_position_.y(),
        uav_current_state_.position.z(), ground_reference_z_,
        static_cast<unsigned>(land_type_), land_yaw_);
  }

  if (land_type_ == 1U) {
    temp_output.clear_all();
    return temp_output;
  }

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

  temp_output.channel_enable(ControllerOutputMask::VELOCITY);
  temp_output.channel_enable(ControllerOutputMask::YAW);
  temp_output.velocity.setZero();
  temp_output.yaw = land_yaw_;

  if (landed_detected) {
    if (land_touchdown_detected_time_.isZero()) {
      land_touchdown_detected_time_ = now;
      ROS_INFO(
          "[Position_Controller] LAND touchdown detected: current=(%.3f, "
          "%.3f, %.3f) sensor=%d low_velocity=%d",
          uav_current_state_.position.x(), uav_current_state_.position.y(),
          uav_current_state_.position.z(), px4_land_status_ ? 1 : 0,
          landed_by_velocity ? 1 : 0);
    }

    temp_output.velocity.z() = -std::abs(land_touchdown_downpress_speed_mps_);
    return temp_output;
  }

  const double ex = land_expect_position_.x() - uav_current_state_.position.x();
  const double ey = land_expect_position_.y() - uav_current_state_.position.y();
  const double descent_speed =
      (land_max_velocity_ < 0.0) ? land_max_velocity_
                                 : -std::max(0.1, land_max_velocity_);

  temp_output.velocity.x() =
      clampSymmetric(land_xy_kp_ * ex, land_max_velocity_xy_mps_);
  temp_output.velocity.y() =
      clampSymmetric(land_xy_kp_ * ey, land_max_velocity_xy_mps_);
  temp_output.velocity.z() = descent_speed;
  return temp_output;
}

} // namespace uav_control
