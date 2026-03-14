#include "sunray_statemachine/sunray_statemachine.h"

#include <cmath>

namespace {
// 从四元数中得到yaw角 
double yaw_from_quat(const Eigen::Quaterniond &q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}
} // namespace

namespace sunray_fsm {

// 起飞服务
bool Sunray_StateMachine::takeoff_srv_cb(
    uav_control::Takeoff::Request &req,
    uav_control::Takeoff::Response &res) {
  	
		// 首先如果消息传入值大于0就使用消息传入值，消息传入值小于或者等于零就使用配置文件中的值
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
		if (req.takeoff_relative_height > 0.0) {
      fsm_param_config_.takeoff_height_m = req.takeoff_relative_height;
    }
    if (req.takeoff_max_velocity > 0.0) {
      fsm_param_config_.takeoff_max_vel_mps = req.takeoff_max_velocity;
    }
  }
		// 返回事件入队结果
  const bool result = queue_event(SunrayEvent::TAKEOFF_REQUEST);
  if (result) {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    clear_active_control_source_locked();
  }
  res.accepted = result;
  res.message = result ? "takeoff request queued" : "takeoff request rejected";
  // 这里返回的true主要是根据ros的惯例，返回的并不是这个服务所要实现的请求是否被实现，而是这个请求是否被正常接受到了
	return true;
}

// 降落服务
bool Sunray_StateMachine::land_srv_cb(uav_control::Land::Request &req,
                                      uav_control::Land::Response &res) {
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (req.land_type >= 0) {
      fsm_param_config_.land_type = req.land_type;
    }
    if (req.land_max_velocity > 0.0) {
      fsm_param_config_.land_max_vel_mps = req.land_max_velocity;
    }
  }

  const bool ok = queue_event(SunrayEvent::LAND_REQUEST);
  if (ok) {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    clear_active_control_source_locked();
  }
  res.accepted = ok;
  res.message = ok ? "land request queued" : "land request rejected";
  return true;
}

bool Sunray_StateMachine::return_srv_cb(
    uav_control::ReturnHome::Request &req,
    uav_control::ReturnHome::Response &res) {
  bool request_valid = true;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (req.land_max_velocity > 0.0) {
      fsm_param_config_.land_max_vel_mps = req.land_max_velocity;
    }
    return_use_takeoff_homepoint_ = req.use_takeoff_homepoint;
    return_target_position_ = req.target_position;
    return_target_yaw_ctrl_ = req.yaw_ctrl;
    return_target_yaw_ = req.yaw;
    uav_control::TrajectoryPoint preview_target;
    request_valid = build_return_target_locked(&preview_target);
  }
  if (!request_valid) {
    res.accepted = false;
    res.message = "return request rejected: no valid homepoint/target";
    return true;
  }
  const bool ok = queue_event(SunrayEvent::RETURN_REQUEST);
  if (ok) {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    clear_active_control_source_locked();
  }
  res.accepted = ok;
  res.message = ok ? "return request queued" : "return request rejected";
  return true;
}

bool Sunray_StateMachine::position_srv_cb(
    uav_control::PositionRequest::Request &req,
    uav_control::PositionRequest::Response &res) {
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (!sunray_controller_) {
      res.accepted = false;
      res.message = "position request rejected: no controller";
      return true;
    }
    uav_control::TrajectoryPoint traj;
    uav_control::TrajectoryPointReference traj_ref;
    traj_ref.set_position(Eigen::Vector3d(req.target_position.x,
                                          req.target_position.y,
                                          req.target_position.z));
    if (req.yaw_ctrl) {
      traj_ref.set_yaw(req.yaw);
    }
    traj = traj_ref.toRosMessage();
    (void)sunray_controller_->set_trajectory(traj);
  }
  const bool ok = queue_event(SunrayEvent::ENTER_POSITION_CONTROL);
  if (ok) {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    clear_active_control_source_locked();
  }
  res.accepted = ok;
  res.message = ok ? "position request queued" : "position request rejected";
  return true;
}

void Sunray_StateMachine::velocity_cmd_envelope_cb(
    const uav_control::VelocityCmdEnvelope::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  ros::Time stamp;
  double timeout_s = 0.0;
  int priority = 0;
  std::string reject_reason;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (!sunray_controller_) {
      ROS_WARN_THROTTLE(1.0,
                        "[SunrayFSM] velocity_cmd_envelope ignored: "
                        "no controller");
      return;
    }
    if (!accept_control_meta_locked(msg->meta, SunrayState::VELOCITY_CONTROL,
                                    &stamp, &timeout_s, &priority,
                                    &reject_reason)) {
      ROS_WARN_THROTTLE(
          1.0,
          "[SunrayFSM] velocity_cmd_envelope rejected: source=%u reason=%s",
          static_cast<unsigned>(msg->meta.source_id), reject_reason.c_str());
      return;
    }

    uav_control::TrajectoryPoint traj;
    uav_control::TrajectoryPointReference traj_ref;
    if (msg->payload.position_ctrl) {
      traj_ref.set_position(Eigen::Vector3d(msg->payload.target_position.x,
                                            msg->payload.target_position.y,
                                            msg->payload.target_position.z));
    }
    traj_ref.set_velocity(Eigen::Vector3d(msg->payload.target_linear_velocity.x,
                                          msg->payload.target_linear_velocity.y,
                                          msg->payload.target_linear_velocity.z));
    traj_ref.set_yaw_rate(msg->payload.target_angular_velocity.z);
    traj = traj_ref.toRosMessage();
    (void)sunray_controller_->set_trajectory(traj);
    update_active_control_source_locked(msg->meta,
                                        SunrayState::VELOCITY_CONTROL, stamp,
                                        timeout_s, priority);
  }
  (void)queue_event(SunrayEvent::ENTER_VELOCITY_CONTROL);
}

void Sunray_StateMachine::attitude_cmd_envelope_cb(
    const uav_control::AttitudeCmdEnvelope::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  ros::Time stamp;
  double timeout_s = 0.0;
  int priority = 0;
  std::string reject_reason;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (!sunray_controller_) {
      ROS_WARN_THROTTLE(1.0,
                        "[SunrayFSM] attitude_cmd_envelope ignored: "
                        "no controller");
      return;
    }
    if (!accept_control_meta_locked(msg->meta, SunrayState::ATTITUDE_CONTROL,
                                    &stamp, &timeout_s, &priority,
                                    &reject_reason)) {
      ROS_WARN_THROTTLE(
          1.0,
          "[SunrayFSM] attitude_cmd_envelope rejected: source=%u reason=%s",
          static_cast<unsigned>(msg->meta.source_id), reject_reason.c_str());
      return;
    }

    double yaw = msg->payload.yaw;
    if (!msg->payload.yaw_ctrl_types) {
      const auto &state = sunray_controller_->get_current_state();
      yaw += yaw_from_quat(state.orientation);
    }
    uav_control::TrajectoryPoint traj;
    uav_control::TrajectoryPointReference traj_ref;
    traj_ref.set_position(sunray_controller_->get_current_state().position);
    traj_ref.set_yaw(yaw);
    traj = traj_ref.toRosMessage();
    (void)sunray_controller_->set_trajectory(traj);
    update_active_control_source_locked(msg->meta,
                                        SunrayState::ATTITUDE_CONTROL, stamp,
                                        timeout_s, priority);
  }
  (void)queue_event(SunrayEvent::ENTER_ATTITUDE_CONTROL);
}

void Sunray_StateMachine::trajectory_envelope_cb(
    const uav_control::TrajectoryEnvelope::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  ros::Time stamp;
  double timeout_s = 0.0;
  int priority = 0;
  std::string reject_reason;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (!sunray_controller_) {
      ROS_WARN_THROTTLE(1.0,
                        "[SunrayFSM] trajectory_envelope ignored: "
                        "no controller");
      return;
    }
    if (!accept_control_meta_locked(msg->meta, SunrayState::TRAJECTORY_CONTROL,
                                    &stamp, &timeout_s, &priority,
                                    &reject_reason)) {
      ROS_WARN_THROTTLE(
          1.0,
          "[SunrayFSM] trajectory_envelope rejected: source=%u reason=%s",
          static_cast<unsigned>(msg->meta.source_id), reject_reason.c_str());
      return;
    }
    if (!sunray_controller_->set_trajectory(msg->payload)) {
      ROS_WARN_THROTTLE(1.0,
                        "[SunrayFSM] trajectory_envelope rejected by controller");
      return;
    }
    update_active_control_source_locked(msg->meta,
                                        SunrayState::TRAJECTORY_CONTROL, stamp,
                                        timeout_s, priority);
  }
  (void)queue_event(SunrayEvent::ENTER_TRAJECTORY_CONTROL);
}

void Sunray_StateMachine::complex_cmd_envelope_cb(
    const uav_control::ComplexCmdEnvelope::ConstPtr &msg) {
  if (!msg) {
    return;
  }

  ros::Time stamp;
  double timeout_s = 0.0;
  int priority = 0;
  std::string reject_reason;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    if (!accept_control_meta_locked(msg->meta, SunrayState::COMPLEX_CONTROL,
                                    &stamp, &timeout_s, &priority,
                                    &reject_reason)) {
      ROS_WARN_THROTTLE(
          1.0,
          "[SunrayFSM] complex_cmd_envelope rejected: source=%u reason=%s",
          static_cast<unsigned>(msg->meta.source_id), reject_reason.c_str());
      return;
    }
    update_active_control_source_locked(msg->meta, SunrayState::COMPLEX_CONTROL,
                                        stamp, timeout_s, priority);
  }

  // TODO: direct MAVROS passthrough is not implemented in FSM yet.
  ROS_WARN_THROTTLE(
      1.0, "[SunrayFSM] complex_cmd_envelope received but passthrough not wired");
  (void)queue_event(SunrayEvent::ENTER_COMPLEX_CONTROL);
}

} // namespace sunray_fsm
