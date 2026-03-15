#include "sunray_statemachine/sunray_statemachine.h"
#include "controller/px4_position_controller/px4_position_controller.h"
#include "controller/sunray_attitude_controller/sunray_attitude_controller.hpp"
#include <algorithm>
#include <cmath>
namespace {
template <typename CovarianceArray>
bool has_meaningful_covariance(const CovarianceArray &cov) {
  for (const double v : cov) {
    if (std::isfinite(v) && std::abs(v) > 1e-9) {
      return true;
    }
  }
  return false;
}

bool is_finite_vector3(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

constexpr double kReturnHoverBeforeLandS = 1.0;
} // namespace

namespace sunray_fsm {

// 状态机构造函数，引用ros句柄，读取sunray_control_config.yaml中的参数
Sunray_StateMachine::Sunray_StateMachine(ros::NodeHandle &nh)
    : nh_(nh), uav_ns_(resolve_uav_namespace()),
      fsm_current_state_(SunrayState::OFF), fsm_param_config_{},
      external_odom_sub_(), latest_external_odom_{},
      latest_external_odom_msg_{}, odom_to_px4_vision_pose_pub_(),
      odom_to_px4_odometry_pub_(), odom_fuse_timer_(), px4_data_reader_(nh_),
      px4_param_manager_(nh_), px4_arming_client_(), px4_set_mode_client_(),
      px4_offboard_retry_state_{}, sunray_controller_(nullptr),
      controller_update_timer_(), arbiter_() {
  // 如果uav_ns命名空间为空，则在ros全局参数空间中查找参数/uav_name与/uav_id，然后拼接uav_ns命名空间
  if (uav_ns_.empty()) {
    std::string fallback_uav_name = "uav";
    int fallback_uav_id = 1;
    nh_.param("/uav_name", fallback_uav_name, fallback_uav_name);
    nh_.param("/uav_id", fallback_uav_id, fallback_uav_id);
    uav_ns_ = fallback_uav_name + std::to_string(fallback_uav_id);
  }
  // 构造config_ns命名空间，用于读取在uav_name+uav_id命名空间下的参数
  const std::string config_ns = uav_ns_.empty() ? "" : ("/" + uav_ns_);
  ros::NodeHandle cfg_nh = config_ns.empty() ? nh_ : ros::NodeHandle(config_ns);
  // 构造ctrl命名空间，用于发布控制参数
  const std::string ctrl_ns = "/" + uav_ns_ + "/sunray_control";
  ctrl_nh_ = ros::NodeHandle(ctrl_ns);

  // 1) 读取参数，顺序与sunray_control_config.yaml一致
  // -------------------基本参数-----------------------
  cfg_nh.param("uav_name", fsm_param_config_.uav_name,
               fsm_param_config_.uav_name);
  cfg_nh.param("uav_id", fsm_param_config_.uav_id, fsm_param_config_.uav_id);
  cfg_nh.param("mass_kg", fsm_param_config_.mass_kg, fsm_param_config_.mass_kg);
  cfg_nh.param("gravity", fsm_param_config_.gravity, fsm_param_config_.gravity);

  cfg_nh.param("controller_types", fsm_param_config_.controller_type,
               fsm_param_config_.controller_type);
  cfg_nh.param("controller_update_frequency",
               fsm_param_config_.controller_update_hz,
               fsm_param_config_.controller_update_hz);
  cfg_nh.param("supervisor_update_frequency",
               fsm_param_config_.supervisor_update_hz,
               fsm_param_config_.supervisor_update_hz);

  cfg_nh.param("odom_topic_name", fsm_param_config_.odom_topic_name,
               fsm_param_config_.odom_topic_name);
  cfg_nh.param("fuse_odom_to_px4", fsm_param_config_.fuse_odom_to_px4,
               fsm_param_config_.fuse_odom_to_px4);
  cfg_nh.param("fuse_odom_type", fsm_param_config_.fuse_odom_type,
               fsm_param_config_.fuse_odom_type);
  cfg_nh.param("fuse_odom_frequency", fsm_param_config_.fuse_odom_frequency_hz,
               fsm_param_config_.fuse_odom_frequency_hz);

  // -------------------保护参数-----------------------
  cfg_nh.param("low_voltage", fsm_param_config_.low_voltage_v,
               fsm_param_config_.low_voltage_v);
  cfg_nh.param("low_voltage_operate", fsm_param_config_.low_voltage_action,
               fsm_param_config_.low_voltage_action);
  cfg_nh.param("control_with_no_rc", fsm_param_config_.control_with_no_rc,
               fsm_param_config_.control_with_no_rc);
  cfg_nh.param("lost_with_rc", fsm_param_config_.lost_with_rc_action,
               fsm_param_config_.lost_with_rc_action);
  cfg_nh.param("arm_with_code", fsm_param_config_.arm_with_code,
               fsm_param_config_.arm_with_code);
  cfg_nh.param("takeoff_with_code", fsm_param_config_.takeoff_with_code,
               fsm_param_config_.takeoff_with_code);
  cfg_nh.param("check_flip", fsm_param_config_.check_flip,
               fsm_param_config_.check_flip);
  cfg_nh.param("tilt_angle_max", fsm_param_config_.tilt_angle_max_deg,
               fsm_param_config_.tilt_angle_max_deg);

  // -------------------起飞降落参数-----------------------
  cfg_nh.param("takeoff_relative_height", fsm_param_config_.takeoff_height_m,
               fsm_param_config_.takeoff_height_m);
  cfg_nh.param("takeoff_max_velocity", fsm_param_config_.takeoff_max_vel_mps,
               fsm_param_config_.takeoff_max_vel_mps);
  cfg_nh.param("land_type", fsm_param_config_.land_type,
               fsm_param_config_.land_type);
  cfg_nh.param("land_max_velocity", fsm_param_config_.land_max_vel_mps,
               fsm_param_config_.land_max_vel_mps);

  // -------------------电子围栏参数-----------------------
  cfg_nh.param("electronic_fence/x_max", fsm_param_config_.fence_x_max,
               fsm_param_config_.fence_x_max);
  cfg_nh.param("electronic_fence/x_min", fsm_param_config_.fence_x_min,
               fsm_param_config_.fence_x_min);
  cfg_nh.param("electronic_fence/y_max", fsm_param_config_.fence_y_max,
               fsm_param_config_.fence_y_max);
  cfg_nh.param("electronic_fence/y_min", fsm_param_config_.fence_y_min,
               fsm_param_config_.fence_y_min);
  cfg_nh.param("electronic_fence/z_max", fsm_param_config_.fence_z_max,
               fsm_param_config_.fence_z_max);
  cfg_nh.param("electronic_fence/z_min", fsm_param_config_.fence_z_min,
               fsm_param_config_.fence_z_min);

  // -------------------消息超时参数-----------------------
  cfg_nh.param("msg_timeout/odom", fsm_param_config_.timeout_odom_s,
               fsm_param_config_.timeout_odom_s);
  cfg_nh.param("msg_timeout/rc", fsm_param_config_.timeout_rc_s,
               fsm_param_config_.timeout_rc_s);
  cfg_nh.param("msg_timeout/control_heartbeat",
               fsm_param_config_.timeout_control_hb_s,
               fsm_param_config_.timeout_control_hb_s);
  cfg_nh.param("msg_timeout/imu", fsm_param_config_.timeout_imu_s,
               fsm_param_config_.timeout_imu_s);
  cfg_nh.param("msg_timeout/battery", fsm_param_config_.timeout_battery_s,
               fsm_param_config_.timeout_battery_s);
  // -------------------位置误差参数-----------------------
  cfg_nh.param("error_tolerance/pos_x",
               fsm_param_config_.error_tolerance_pos_x_m,
               fsm_param_config_.error_tolerance_pos_x_m);
  cfg_nh.param("error_tolerance/pos_y",
               fsm_param_config_.error_tolerance_pos_y_m,
               fsm_param_config_.error_tolerance_pos_y_m);
  cfg_nh.param("error_tolerance/pos_z",
               fsm_param_config_.error_tolerance_pos_z_m,
               fsm_param_config_.error_tolerance_pos_z_m);
  // -------------------飞行速度参数-----------------------
  cfg_nh.param("max_velocity/x_vel", fsm_param_config_.max_velocity_x_mps,
               fsm_param_config_.max_velocity_x_mps);
  cfg_nh.param("max_velocity/y_vel", fsm_param_config_.max_velocity_y_mps,
               fsm_param_config_.max_velocity_y_mps);
  cfg_nh.param("max_velocity/z_vel", fsm_param_config_.max_velocity_z_mps,
               fsm_param_config_.max_velocity_z_mps);
  cfg_nh.param("max_velocity_with_rc/x_vel",
               fsm_param_config_.max_velocity_with_rc_x_mps,
               fsm_param_config_.max_velocity_with_rc_x_mps);
  cfg_nh.param("max_velocity_with_rc/y_vel",
               fsm_param_config_.max_velocity_with_rc_y_mps,
               fsm_param_config_.max_velocity_with_rc_y_mps);
  cfg_nh.param("max_velocity_with_rc/z_vel",
               fsm_param_config_.max_velocity_with_rc_z_mps,
               fsm_param_config_.max_velocity_with_rc_z_mps);
  load_control_source_policies(cfg_nh);

  // 2) 初始化 MAVROS service client
  const std::string mavros_ns =
      uav_ns_.empty() ? "/mavros" : ("/" + uav_ns_ + "/mavros");
  px4_arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");
  px4_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
  odom_to_px4_vision_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      mavros_ns + "/vision_pose/pose", 20);
  odom_to_px4_odometry_pub_ =
      nh_.advertise<nav_msgs::Odometry>(mavros_ns + "/odometry/in", 20);
  external_odom_sub_ =
      nh_.subscribe(fsm_param_config_.odom_topic_name, 20,
                    &Sunray_StateMachine::external_odom_cb, this);
  odom_fuse_timer_ = nh_.createTimer(
      ros::Duration(1.0 / fsm_param_config_.fuse_odom_frequency_hz),
      &Sunray_StateMachine::odom_fuse_timer_cb, this);

  // 3) 初始化仲裁器
  if (!arbiter_.init(nh_)) {
    ROS_WARN("[SunrayFSM] arbiter init failed");
  }

  // 4) 注册控制器 + 启动周期 update 定时器
  (void)register_controller(fsm_param_config_.controller_type);

  const double hz = std::max(50.0, fsm_param_config_.controller_update_hz);
  controller_update_timer_ =
      nh_.createTimer(ros::Duration(1.0 / hz),
                      &Sunray_StateMachine::controller_update_timer_cb, this);
  
	// 5) 发布对应控制接口(先占位置)
  // 话题订阅
  velocity_cmd_envelope_sub_ = ctrl_nh_.subscribe(
      "velocity_cmd_envelope", 10,
      &Sunray_StateMachine::velocity_cmd_envelope_cb, this);
  attitude_cmd_envelope_sub_ = ctrl_nh_.subscribe(
      "attitude_cmd_envelope", 10,
      &Sunray_StateMachine::attitude_cmd_envelope_cb, this);
  trajectory_envelope_sub_ = ctrl_nh_.subscribe(
      "trajectory_envelope", 10,
      &Sunray_StateMachine::trajectory_envelope_cb, this);
  complex_cmd_envelope_sub_ = ctrl_nh_.subscribe(
      "complex_cmd_envelope", 10,
      &Sunray_StateMachine::complex_cmd_envelope_cb, this);

  // 服务
  takeoff_srv_ = ctrl_nh_.advertiseService(
      "takeoff_request", &Sunray_StateMachine::takeoff_srv_cb, this);
  land_srv_ = ctrl_nh_.advertiseService(
      "land_request", &Sunray_StateMachine::land_srv_cb, this);
  return_srv_ = ctrl_nh_.advertiseService(
      "return_request", &Sunray_StateMachine::return_srv_cb, this);
  position_srv_ = ctrl_nh_.advertiseService(
      "position_request", &Sunray_StateMachine::position_srv_cb, this);
  fsm_state_pub_ = ctrl_nh_.advertise<std_msgs::UInt8>("fsm_state", 1, true);
  publish_fsm_state();
  
	// 
		ROS_INFO("[SunrayFSM] init done, uav_ns='%s', state=OFF, odom='%s'",
           uav_ns_.c_str(), fsm_param_config_.odom_topic_name.c_str());
}

void Sunray_StateMachine::load_control_source_policies(ros::NodeHandle &cfg_nh) {
  control_source_policies_.clear();

  ControlSourcePolicy default_policy;
  cfg_nh.param("control_sources/default/priority", default_policy.priority, 0);
  cfg_nh.param("control_sources/default/timeout", default_policy.timeout_s,
               fsm_param_config_.timeout_control_hb_s);
  control_source_policies_[uav_control::ControlMeta::SOURCE_UNSPECIFIED] =
      default_policy;

  const auto load_policy = [&](uint8_t source_id, const std::string &name,
                               int default_priority,
                               double default_timeout_s) {
    ControlSourcePolicy policy;
    cfg_nh.param("control_sources/" + name + "/priority", policy.priority,
                 default_priority);
    cfg_nh.param("control_sources/" + name + "/timeout", policy.timeout_s,
                 default_timeout_s);
    control_source_policies_[source_id] = policy;
  };

  load_policy(uav_control::ControlMeta::SOURCE_API, "api", 20,
              fsm_param_config_.timeout_control_hb_s);
  load_policy(uav_control::ControlMeta::SOURCE_MISSION, "mission", 50,
              fsm_param_config_.timeout_control_hb_s);
  load_policy(uav_control::ControlMeta::SOURCE_PLANNER, "planner", 40,
              fsm_param_config_.timeout_control_hb_s);
  load_policy(uav_control::ControlMeta::SOURCE_RC, "rc", 100,
              fsm_param_config_.timeout_control_hb_s);
  load_policy(uav_control::ControlMeta::SOURCE_SAFETY, "safety", 255,
              fsm_param_config_.timeout_control_hb_s);
}

bool Sunray_StateMachine::is_active_control_source_expired_locked(
    const ros::Time &now) const {
  if (!active_control_source_.valid || active_control_source_.timeout_s <= 0.0 ||
      active_control_source_.last_update_time.isZero()) {
    return false;
  }
  return (now - active_control_source_.last_update_time).toSec() >
         active_control_source_.timeout_s;
}

void Sunray_StateMachine::clear_active_control_source_locked() {
  active_control_source_ = ActiveControlSource();
}

void Sunray_StateMachine::update_px4_landed_hold_locked(
    SunrayState state, bool px4_landed, const ros::Time &now) {
  const bool landing_state = (state == SunrayState::LAND ||
                              state == SunrayState::EMERGENCY_LAND);
  if (!landing_state || !px4_landed) {
    px4_landed_hold_start_time_ = ros::Time(0);
    return;
  }

  if (px4_landed_hold_start_time_.isZero() || now < px4_landed_hold_start_time_) {
    px4_landed_hold_start_time_ = now;
  }
}

void Sunray_StateMachine::update_controller_touchdown_hold_locked(
    SunrayState state, bool touchdown_detected, const ros::Time &now) {
  const bool landing_state = (state == SunrayState::LAND ||
                              state == SunrayState::EMERGENCY_LAND);
  if (!landing_state || !touchdown_detected) {
    controller_touchdown_hold_start_time_ = ros::Time(0);
    return;
  }

  if (controller_touchdown_hold_start_time_.isZero() ||
      now < controller_touchdown_hold_start_time_) {
    controller_touchdown_hold_start_time_ = now;
  }
}

bool Sunray_StateMachine::is_px4_landed_hold_satisfied_locked(
    const ros::Time &now) const {
  return get_px4_landed_hold_elapsed_s_locked(now) >=
         px4_landed_hold_required_s_;
}

bool Sunray_StateMachine::is_controller_touchdown_hold_satisfied_locked(
    const ros::Time &now) const {
  return get_controller_touchdown_hold_elapsed_s_locked(now) >=
         px4_landed_hold_required_s_;
}

double Sunray_StateMachine::get_px4_landed_hold_elapsed_s_locked(
    const ros::Time &now) const {
  if (px4_landed_hold_start_time_.isZero() || now < px4_landed_hold_start_time_) {
    return 0.0;
  }
  return (now - px4_landed_hold_start_time_).toSec();
}

double Sunray_StateMachine::get_controller_touchdown_hold_elapsed_s_locked(
    const ros::Time &now) const {
  if (controller_touchdown_hold_start_time_.isZero() ||
      now < controller_touchdown_hold_start_time_) {
    return 0.0;
  }
  return (now - controller_touchdown_hold_start_time_).toSec();
}

bool Sunray_StateMachine::accept_control_meta_locked(
    const uav_control::ControlMeta &meta, SunrayState requested_state,
    ros::Time *stamp, double *timeout_s, int *priority, std::string *reason) {
  const ros::Time now = ros::Time::now();
  if (is_active_control_source_expired_locked(now)) {
    clear_active_control_source_locked();
  }

  const ros::Time resolved_stamp = meta.header.stamp.isZero()
                                       ? now
                                       : meta.header.stamp;
  const auto default_it = control_source_policies_.find(
      uav_control::ControlMeta::SOURCE_UNSPECIFIED);
  const ControlSourcePolicy default_policy =
      (default_it != control_source_policies_.end())
          ? default_it->second
          : ControlSourcePolicy();
  const auto policy_it = control_source_policies_.find(meta.source_id);
  const ControlSourcePolicy policy =
      (policy_it != control_source_policies_.end()) ? policy_it->second
                                                    : default_policy;
  const double resolved_timeout_s =
      (meta.timeout.toSec() > 0.0) ? meta.timeout.toSec() : policy.timeout_s;

  if (stamp) {
    *stamp = resolved_stamp;
  }
  if (timeout_s) {
    *timeout_s = resolved_timeout_s;
  }
  if (priority) {
    *priority = policy.priority;
  }

  if (!active_control_source_.valid) {
    return true;
  }

  if (meta.source_id == active_control_source_.source_id) {
    if (!meta.replace_same_source) {
      if (reason) {
        *reason = "same source replacement disabled";
      }
      return false;
    }
    if (meta.sequence_id != 0U && active_control_source_.sequence_id != 0U &&
        meta.sequence_id < active_control_source_.sequence_id) {
      if (reason) {
        *reason = "stale sequence id";
      }
      return false;
    }
    return true;
  }

  if (policy.priority > active_control_source_.priority && meta.allow_preempt) {
    return true;
  }

  if (reason) {
    *reason = "blocked by active higher-priority control source";
  }
  (void)requested_state;
  return false;
}

void Sunray_StateMachine::update_active_control_source_locked(
    const uav_control::ControlMeta &meta, SunrayState requested_state,
    const ros::Time &stamp, double timeout_s, int priority) {
  active_control_source_.valid = true;
  active_control_source_.source_id = meta.source_id;
  active_control_source_.sequence_id = meta.sequence_id;
  active_control_source_.priority = priority;
  active_control_source_.last_update_time = stamp;
  active_control_source_.timeout_s = timeout_s;
  active_control_source_.control_state = requested_state;
}

bool Sunray_StateMachine::register_controller(int controller_types) {
  // controller_type 约定（与 yaml 对齐）:
  // 0: PX4 position controller（已实现）
  // 1: px4_velocity_controller（预留）
  // 2: sunray_attitude_controller #2（预留）
  // 3: rl_raptor_controller #3（预留）
  std::shared_ptr<uav_control::Base_Controller> selected_controller;

  switch (controller_types) {
  case 0:
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 1:
    ROS_WARN("[SunrayFSM] controller_type=1 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  case 2:
    selected_controller = std::make_shared<uav_control::Attitude_Controller>();
    break;
  case 3:
    ROS_WARN("[SunrayFSM] controller_type=3 reserved, fallback to type=0 "
             "(PX4 position controller)");
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  default:
    ROS_WARN("[SunrayFSM] unknown controller_type=%d, fallback to type=0 "
             "(PX4 position controller)",
             controller_types);
    selected_controller = std::make_shared<uav_control::Position_Controller>();
    break;
  }

  if (!selected_controller) {
    ROS_ERROR("[SunrayFSM] register_controller failed: create controller "
              "instance failed (type=%d)",
              controller_types);
    return false;
  }

  if (controller_types == 2) {
    const std::string config_ns = uav_ns_.empty() ? "" : ("/" + uav_ns_);
    ros::NodeHandle cfg_nh =
        config_ns.empty() ? nh_ : ros::NodeHandle(config_ns);
    std::shared_ptr<uav_control::Attitude_Controller> attitude_controller =
        std::dynamic_pointer_cast<uav_control::Attitude_Controller>(
            selected_controller);
    if (!attitude_controller || !attitude_controller->load_param(cfg_nh)) {
      ROS_ERROR("[SunrayFSM] register_controller failed: "
                "sunray_attitude_controller load_param failed");
      return false;
    }
  }

  sunray_controller_ = selected_controller;
  ROS_INFO("[SunrayFSM] register global controller success, type=%d",
           controller_types);
  return true;
}

bool Sunray_StateMachine::resolve_next_state_locked(
    SunrayEvent event, SunrayState *next_state) const {
  if (!next_state) {
    return false;
  }

  switch (fsm_current_state_) {
  case SunrayState::OFF:
    if (event == SunrayEvent::TAKEOFF_REQUEST && can_takeoff_locked()) {
      *next_state = SunrayState::TAKEOFF;
      return true;
    }
    break;

  case SunrayState::TAKEOFF:
    if (event == SunrayEvent::TAKEOFF_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::HOVER:
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      *next_state = SunrayState::RETURN;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    if (event == SunrayEvent::ENTER_POSITION_CONTROL) {
      *next_state = SunrayState::POSITION_CONTROL;
      return true;
    }
    if (event == SunrayEvent::ENTER_VELOCITY_CONTROL) {
      *next_state = SunrayState::VELOCITY_CONTROL;
      return true;
    }
    if (event == SunrayEvent::ENTER_ATTITUDE_CONTROL) {
      *next_state = SunrayState::ATTITUDE_CONTROL;
      return true;
    }
    if (event == SunrayEvent::ENTER_COMPLEX_CONTROL) {
      *next_state = SunrayState::COMPLEX_CONTROL;
      return true;
    }
    if (event == SunrayEvent::ENTER_TRAJECTORY_CONTROL) {
      *next_state = SunrayState::TRAJECTORY_CONTROL;
      return true;
    }
    break;

  case SunrayState::RETURN:
    if (event == SunrayEvent::RETURN_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::LAND:
    if (event == SunrayEvent::LAND_COMPLETED) {
      *next_state = SunrayState::OFF;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::EMERGENCY_LAND:
    if (event == SunrayEvent::EMERGENCY_COMPLETED) {
      *next_state = SunrayState::OFF;
      return true;
    }
    break;

  case SunrayState::POSITION_CONTROL:
    if (event == SunrayEvent::ENTER_POSITION_CONTROL) {
      *next_state = SunrayState::POSITION_CONTROL;
      return true;
    }
    if (event == SunrayEvent::POSITION_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      *next_state = SunrayState::RETURN;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::VELOCITY_CONTROL:
    if (event == SunrayEvent::ENTER_VELOCITY_CONTROL) {
      *next_state = SunrayState::VELOCITY_CONTROL;
      return true;
    }
    if (event == SunrayEvent::VELOCITY_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      *next_state = SunrayState::RETURN;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::ATTITUDE_CONTROL:
    if (event == SunrayEvent::ENTER_ATTITUDE_CONTROL) {
      *next_state = SunrayState::ATTITUDE_CONTROL;
      return true;
    }
    if (event == SunrayEvent::ATTITUDE_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      *next_state = SunrayState::RETURN;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::COMPLEX_CONTROL:
    if (event == SunrayEvent::ENTER_COMPLEX_CONTROL) {
      *next_state = SunrayState::COMPLEX_CONTROL;
      return true;
    }
    if (event == SunrayEvent::COMPLEX_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      *next_state = SunrayState::RETURN;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;

  case SunrayState::TRAJECTORY_CONTROL:
    if (event == SunrayEvent::ENTER_TRAJECTORY_CONTROL) {
      *next_state = SunrayState::TRAJECTORY_CONTROL;
      return true;
    }
    if (event == SunrayEvent::TRAJECTORY_COMPLETED) {
      *next_state = SunrayState::HOVER;
      return true;
    }
    if (event == SunrayEvent::LAND_REQUEST) {
      *next_state = SunrayState::LAND;
      return true;
    }
    if (event == SunrayEvent::RETURN_REQUEST) {
      *next_state = SunrayState::RETURN;
      return true;
    }
    if (event == SunrayEvent::WATCHDOG_ERROR ||
        event == SunrayEvent::EMERGENCY_REQUEST) {
      *next_state = SunrayState::EMERGENCY_LAND;
      return true;
    }
    break;
  }

  return false;
}

bool Sunray_StateMachine::queue_event(SunrayEvent event) {
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    SunrayState next_state = fsm_current_state_;
    if (!resolve_next_state_locked(event, &next_state)) {
      return false;
    }
  }

  std::lock_guard<std::mutex> event_lock(event_mutex_);
  if (!pending_events_.empty() && pending_events_.back() == event) {
    return true;
  }
  pending_events_.push_back(event);
  return true;
}

bool Sunray_StateMachine::handle_event(SunrayEvent event) {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  const SunrayState prev = fsm_current_state_;
  SunrayState next_state = prev;
  if (!resolve_next_state_locked(event, &next_state)) {
    ROS_WARN("[SunrayFSM] ignore event %s at state %s", to_string(event),
             to_string(prev));
    return false;
  }
  return transition_to_locked(next_state);
}

void Sunray_StateMachine::process_pending_events() {
  std::deque<SunrayEvent> pending_events;
  {
    std::lock_guard<std::mutex> event_lock(event_mutex_);
    pending_events.swap(pending_events_);
  }

  for (const SunrayEvent event : pending_events) {
    (void)handle_event(event);
  }
}
/** -----------------状态机更新函数--------------------- */
/**
简单的留存一下思路：状态机的更新函数分为两个，一个负责高频一个负责低频
1. 低频的更新函数负责什么:


1..高频更新函数负责什么：高频更新函数通过状态机中的200hz定时器进行调用，主要负责
	1.1 将高频的里程计注入到控制器
	1.2 获取到控制器的输出
	1.3 这里存在一个问题就是


*/
void Sunray_StateMachine::update() {
  update_slow();
}

void Sunray_StateMachine::update_slow() {
  process_pending_events();

  bool request_emergency = false;
  bool report_odom_timeout = false;
  bool report_control_source_timeout = false;
  bool request_return_completed = false;
  bool request_land_after_return = false;
  bool request_disarm = false;
  bool use_controller_touchdown_fallback = false;
  bool position_completed = false;
  bool trajectory_completed = false;
  double timeout_odom_s = 0.0;
  uint8_t expired_source_id = 0U;
  double expired_timeout_s = 0.0;
  const ros::Time now = ros::Time::now();
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();
  const bool px4_landed =
      px4_state.landed_state == px4_data_types::LandedState::kOnGround;
  bool px4_landed_hold_satisfied = false;
  bool controller_touchdown_hold_satisfied = false;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    update_px4_landed_hold_locked(fsm_current_state_, px4_landed, now);
    const bool controller_touchdown_detected =
        sunray_controller_ && sunray_controller_->is_touchdown_detected();
    update_controller_touchdown_hold_locked(fsm_current_state_,
                                            controller_touchdown_detected, now);
    px4_landed_hold_satisfied = is_px4_landed_hold_satisfied_locked(now);
    controller_touchdown_hold_satisfied =
        is_controller_touchdown_hold_satisfied_locked(now);
    const bool landing_confirmed =
        px4_landed_hold_satisfied || controller_touchdown_hold_satisfied;
    const bool landing_state = (fsm_current_state_ == SunrayState::LAND ||
                                fsm_current_state_ == SunrayState::EMERGENCY_LAND);
    request_disarm = landing_state && landing_confirmed && px4_state.armed;
    use_controller_touchdown_fallback =
        request_disarm && !px4_landed_hold_satisfied &&
        controller_touchdown_hold_satisfied;
    timeout_odom_s = fsm_param_config_.timeout_odom_s;
    if (!check_health_locked() &&
        fsm_current_state_ != SunrayState::EMERGENCY_LAND) {
      request_emergency = true;
    }

    if (!has_fresh_external_odom_locked(ros::Time::now()) &&
        fsm_current_state_ != SunrayState::OFF &&
        fsm_current_state_ != SunrayState::EMERGENCY_LAND) {
      request_emergency = true;
      report_odom_timeout = true;
    }

    if (fsm_current_state_ == SunrayState::RETURN &&
        is_return_target_reached_locked()) {
      request_return_completed = true;
    }

    if (fsm_current_state_ == SunrayState::POSITION_CONTROL &&
        is_position_target_reached_locked()) {
      position_completed = true;
    }

    if (land_after_return_pending_ && fsm_current_state_ == SunrayState::HOVER &&
        !return_hover_start_time_.isZero() &&
        (now - return_hover_start_time_).toSec() >= kReturnHoverBeforeLandS) {
      land_after_return_pending_ = false;
      return_hover_start_time_ = ros::Time(0);
      request_land_after_return = true;
    }
    if (is_active_control_source_expired_locked(now)) {
      report_control_source_timeout = true;
      expired_source_id = active_control_source_.source_id;
      expired_timeout_s = active_control_source_.timeout_s;
      clear_active_control_source_locked();
    }
  }

  if (report_odom_timeout) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] external odom unavailable or timeout (timeout=%.3fs), "
        "queue EMERGENCY",
        timeout_odom_s);
  }

  if (report_control_source_timeout) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] high-rate control source timeout: source=%u timeout=%.3fs",
        static_cast<unsigned>(expired_source_id), expired_timeout_s);
  }

  if (request_emergency) {
    (void)handle_event(SunrayEvent::EMERGENCY_REQUEST);
  }

  if (request_return_completed) {
    Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
    {
      std::lock_guard<std::mutex> lock(fsm_mutex_);
      if (sunray_controller_) {
        current_position = sunray_controller_->get_current_state().position;
      }
      const uav_control::TrajectoryPointReference active_return_target_ref(
          active_return_target_);
      target_position = active_return_target_ref.position;
    }
    ROS_INFO(
        "[SunrayFSM] RETURN completed: current_odom=(%.3f, %.3f, %.3f) "
        "target=(%.3f, %.3f, %.3f)",
        current_position.x(), current_position.y(), current_position.z(),
        target_position.x(), target_position.y(), target_position.z());
    const bool completed = handle_event(SunrayEvent::RETURN_COMPLETED);
    if (completed) {
      std::lock_guard<std::mutex> lock(fsm_mutex_);
      land_after_return_pending_ = true;
      return_hover_start_time_ = now;
    }
  }

  if (request_land_after_return) {
    (void)handle_event(SunrayEvent::LAND_REQUEST);
  }

  if (use_controller_touchdown_fallback) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] PX4 landed_state unavailable, use controller touchdown "
        "fallback for disarm/finalize");
  }

  if (request_disarm && !ensure_disarm()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] waiting for PX4 disarm after landed hold confirmation");
  }

  bool need_auto_land_mode = false;
  bool need_offboard = false;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    need_auto_land_mode = should_use_px4_auto_land_locked();
    need_offboard = requires_offboard_locked();
    const bool landing_confirmed =
        px4_landed_hold_satisfied || controller_touchdown_hold_satisfied;
    const bool landing_finalize_pending =
        (fsm_current_state_ == SunrayState::LAND ||
         fsm_current_state_ == SunrayState::EMERGENCY_LAND) &&
        (landing_confirmed || !px4_state.armed);
    if (landing_finalize_pending) {
      need_auto_land_mode = false;
      need_offboard = false;
    }
  }

  if (need_auto_land_mode) {
    if (!ensure_auto_land_mode()) {
      ROS_WARN_THROTTLE(1.0,
                        "[SunrayFSM] waiting for PX4 AUTO.LAND mode");
    }
  } else if (need_offboard && !ensure_offboard_and_arm()) {
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] waiting for OFFBOARD/ARM before effective control");
  }

  SunrayState fsm_state = SunrayState::OFF;
  bool takeoff_completed = false;
  bool land_completed = false;
  bool emergency_completed = false;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    fsm_state = fsm_current_state_;
    if (sunray_controller_) {
      const bool landing_confirmed =
          px4_landed_hold_satisfied || controller_touchdown_hold_satisfied;
      if (fsm_state == SunrayState::TAKEOFF) {
        takeoff_completed = sunray_controller_->is_takeoff_completed();
      } else if (fsm_state == SunrayState::LAND) {
        land_completed = landing_confirmed && !px4_state.armed;
      } else if (fsm_state == SunrayState::EMERGENCY_LAND) {
        emergency_completed = landing_confirmed && !px4_state.armed;
      } else if (fsm_state == SunrayState::TRAJECTORY_CONTROL) {
        trajectory_completed = sunray_controller_->is_trajectory_completed();
      }
    }
  }

  if (takeoff_completed) {
    (void)handle_event(SunrayEvent::TAKEOFF_COMPLETED);
  } else if (land_completed) {
    (void)handle_event(SunrayEvent::LAND_COMPLETED);
  } else if (emergency_completed) {
    (void)handle_event(SunrayEvent::EMERGENCY_COMPLETED);
  } else if (position_completed) {
    (void)handle_event(SunrayEvent::POSITION_COMPLETED);
  } else if (trajectory_completed) {
    (void)handle_event(SunrayEvent::TRAJECTORY_COMPLETED);
  }

  publish_fsm_state();
}

void Sunray_StateMachine::update_fast() {
  const ros::Time now = ros::Time::now();
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();
  const bool px4_landed =
      px4_state.landed_state == px4_data_types::LandedState::kOnGround;

  std::shared_ptr<uav_control::Base_Controller> controller;
  SunrayState fsm_state = SunrayState::OFF;
  uav_control::UAVStateEstimate controller_state;
  uav_control::ControllerState controller_phase =
      uav_control::ControllerState::UNDEFINED;
  uav_control::ControllerOutput control_output;
  bool external_odom_fresh = false;
  bool auto_land_requested = false;
  bool px4_landed_hold_satisfied = false;
  bool controller_touchdown_hold_satisfied = false;
  bool controller_touchdown_detected = false;
  double px4_landed_hold_elapsed_s = 0.0;
  double controller_touchdown_hold_elapsed_s = 0.0;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    controller = sunray_controller_;
    if (!controller) {
      ROS_WARN_THROTTLE(1.0, "[SunrayFSM] no controller registered");
      return;
    }

    fsm_state = fsm_current_state_;
    update_px4_landed_hold_locked(fsm_state, px4_landed, now);
    px4_landed_hold_satisfied = is_px4_landed_hold_satisfied_locked(now);
    px4_landed_hold_elapsed_s = get_px4_landed_hold_elapsed_s_locked(now);
    (void)controller->set_px4_arm_state(px4_state.armed);
    (void)controller->set_px4_land_status(px4_landed);
    external_odom_fresh = has_fresh_external_odom_locked(now);
    auto_land_requested = should_use_px4_auto_land_locked();
    if (external_odom_fresh) {
      (void)controller->set_current_odom(latest_external_odom_);
    }

    // TODO：无论里程计是否有效，都需要注入px4的姿态
    // (void)controller->set_px4_attitude(const sensor_msgs::Imu &imu_msg);
    control_output = controller->update();
    controller_state = controller->get_current_state();
    controller_phase = controller->get_controller_state();
    controller_touchdown_detected = controller->is_touchdown_detected();
    update_controller_touchdown_hold_locked(fsm_state,
                                            controller_touchdown_detected, now);
    controller_touchdown_hold_satisfied =
        is_controller_touchdown_hold_satisfied_locked(now);
    controller_touchdown_hold_elapsed_s =
        get_controller_touchdown_hold_elapsed_s_locked(now);
  }

  if (!external_odom_fresh &&
      fsm_state != SunrayState::OFF &&
      fsm_state != SunrayState::EMERGENCY_LAND) {
    (void)queue_event(SunrayEvent::EMERGENCY_REQUEST);
  }

  // 有效输出定义为位置、速度、姿态、推力至少有一个是使能的
  const bool has_effective_output =
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::POSITION) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::VELOCITY) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::ATTITUDE) ||
      control_output.is_channel_enabled(
          uav_control::ControllerOutputMask::THRUST);
  // 如果不存在有效输出
  if (!has_effective_output) {
    const bool landing_confirmed =
        px4_landed_hold_satisfied || controller_touchdown_hold_satisfied;
    if (fsm_state == SunrayState::LAND && auto_land_requested) {
      if (landing_confirmed) {
        arbiter_.clear(
            uav_control::Sunray_Control_Arbiter::ControlSource::EXTERNAL);
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "[SunrayFSM] wait landing confirmation before clear at "
            "LAND/AUTO.LAND: px4_landed=%d px4_hold=%.2f/%.2f "
            "ctrl_touchdown=%d ctrl_hold=%.2f/%.2f",
            px4_landed ? 1 : 0, px4_landed_hold_elapsed_s,
            px4_landed_hold_required_s_, controller_touchdown_detected ? 1 : 0,
            controller_touchdown_hold_elapsed_s, px4_landed_hold_required_s_);
      }
      return;
    }
    if ((fsm_state == SunrayState::LAND ||
         fsm_state == SunrayState::EMERGENCY_LAND) &&
        controller_phase == uav_control::ControllerState::OFF) {
      if (landing_confirmed) {
        const auto source =
            (fsm_state == SunrayState::EMERGENCY_LAND)
                ? uav_control::Sunray_Control_Arbiter::ControlSource::EMERGENCY
                : uav_control::Sunray_Control_Arbiter::ControlSource::EXTERNAL;
        arbiter_.clear(source);
      } else {
        ROS_WARN_THROTTLE(
            1.0,
            "[SunrayFSM] controller is OFF in %s, but wait landing "
            "confirmation before clear: px4_landed=%d px4_hold=%.2f/%.2f "
            "ctrl_touchdown=%d ctrl_hold=%.2f/%.2f",
            to_string(fsm_state), px4_landed ? 1 : 0, px4_landed_hold_elapsed_s,
            px4_landed_hold_required_s_, controller_touchdown_detected ? 1 : 0,
            controller_touchdown_hold_elapsed_s, px4_landed_hold_required_s_);
      }
      return;
    }
    // 如果当前控制器的状态为OFF状态，那就没什么事儿，直接结束就行
    if (fsm_state == SunrayState::OFF) {
      return;
    }
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] controller produced empty output at state=%s",
        to_string(fsm_state));
    return;
  }
  // 向仲裁器设置状态机的状态
  arbiter_.set_fsm_state(fsm_state);
  // 向仲裁器设置状态机的状态
  arbiter_.set_uav_state(controller_state);
  // 如果当前处于状态机的紧急降落状态，设置为高优先级(255)
  if (fsm_state == SunrayState::EMERGENCY_LAND) {
    arbiter_.submit(
        uav_control::Sunray_Control_Arbiter::ControlSource::EMERGENCY,
        control_output, ros::Time::now(), 255U);
  } else {
    arbiter_.submit(
        uav_control::Sunray_Control_Arbiter::ControlSource::EXTERNAL,
        control_output, ros::Time::now(), 100U);
  }
  // 仲裁器进行仲裁并输出到mavros中
  (void)arbiter_.arbitrate_and_publish();
}

// 控制器更新回调函数
void Sunray_StateMachine::controller_update_timer_cb(const ros::TimerEvent &) {
  update_fast();
}

// 向px4融合里程计的回调函数
void Sunray_StateMachine::odom_fuse_timer_cb(const ros::TimerEvent &) {
  publish_external_odom_to_px4();
}

// 发布外部里程计数据到px4
void Sunray_StateMachine::publish_external_odom_to_px4() {
  bool fuse_odom_to_px4 = false;
  int fuse_odom_type = 0;
  double timeout_odom_s = 0.0;
  uav_control::UAVStateEstimate latest_external_odom;
  nav_msgs::Odometry latest_external_odom_msg;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    fuse_odom_to_px4 = fsm_param_config_.fuse_odom_to_px4;
    if (!fuse_odom_to_px4 || !latest_external_odom_.isValid()) {
      return;
    }
    fuse_odom_type = fsm_param_config_.fuse_odom_type;
    timeout_odom_s = fsm_param_config_.timeout_odom_s;
    latest_external_odom = latest_external_odom_;
    latest_external_odom_msg = latest_external_odom_msg_;
  }

  const ros::Time now = ros::Time::now();
  // 如果里程计不附带时间戳，就使用当前时间戳
  const ros::Time stamp = latest_external_odom.timestamp.isZero()
                              ? now
                              : latest_external_odom.timestamp;
  const double age_s = (now - stamp).toSec();
  // 校验是否超时
  if (age_s < 0.0 || age_s > timeout_odom_s) {
    ROS_WARN_THROTTLE(1.0,
                      "[SunrayFSM] skip odom->px4 publish: external odom "
                      "timeout, age=%.3f s timeout=%.3f s",
                      age_s, timeout_odom_s);
    return;
  }
  // 根据参数选择通道对里程计进行融合 0:vision_pose 1:odometry
  if (fuse_odom_type == 0) {
    // 使用vision_pose进行融合
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = latest_external_odom_msg.header;
    if (pose_msg.header.stamp.isZero()) {
      pose_msg.header.stamp = now;
    }
    pose_msg.pose = latest_external_odom_msg.pose.pose;
    odom_to_px4_vision_pose_pub_.publish(pose_msg);
  } else if (fuse_odom_type == 1) {
    nav_msgs::Odometry odom_msg = latest_external_odom_msg;
    if (odom_msg.header.stamp.isZero()) {
      odom_msg.header.stamp = now;
    }
    if (odom_msg.child_frame_id.empty()) {
      odom_msg.child_frame_id = "body";
    }
    odom_to_px4_odometry_pub_.publish(odom_msg);
  }
}

// 外部里程计回调
void Sunray_StateMachine::external_odom_cb(
    const nav_msgs::Odometry::ConstPtr &msg) {
  if (!msg) {
    return;
  }
  // 使用UAVStateEstimate结构体快速解析
  uav_control::UAVStateEstimate odom_state(*msg);
  // 如果时间戳为零，则设置为当前时间戳
  if (odom_state.timestamp.isZero()) {
    odom_state.timestamp = ros::Time::now();
  }
  // 如果里程计无效，则结束
  if (!odom_state.isValid()) {
    ROS_WARN_THROTTLE(1.0,
                      "[SunrayFSM] ignore invalid external odom frame='%s'",
                      msg->header.frame_id.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    latest_external_odom_ = odom_state;
    latest_external_odom_msg_ = *msg;
    // 如果时间戳为零，则设置为当前时间戳
    if (latest_external_odom_msg_.header.stamp.isZero()) {
      latest_external_odom_msg_.header.stamp = odom_state.timestamp;
    }
  }
}

// 解算命名空间
std::string Sunray_StateMachine::resolve_uav_namespace() const {
  std::string key;
  std::string ns;
  if (nh_.searchParam("uav_ns", key) && nh_.getParam(key, ns) && !ns.empty()) {
    if (!ns.empty() && ns.front() == '/') {
      return ns.substr(1);
    }
    return ns;
  }

  std::string name;
  int id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh_.searchParam("uav_name", key)) {
    ok_name = nh_.getParam(key, name) && !name.empty();
  }
  if (nh_.searchParam("uav_id", key)) {
    ok_id = nh_.getParam(key, id);
  }
  if (ok_name && ok_id) {
    return name + std::to_string(id);
  }
  return "";
}

bool Sunray_StateMachine::requires_offboard() const {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return requires_offboard_locked();
}

bool Sunray_StateMachine::requires_offboard_locked() const {
  switch (fsm_current_state_) {
  case SunrayState::TAKEOFF:
  case SunrayState::HOVER:
  case SunrayState::RETURN:
  case SunrayState::POSITION_CONTROL:
  case SunrayState::VELOCITY_CONTROL:
  case SunrayState::ATTITUDE_CONTROL:
  case SunrayState::COMPLEX_CONTROL:
  case SunrayState::TRAJECTORY_CONTROL:
    return true;
  case SunrayState::EMERGENCY_LAND:
    if (sunray_controller_ &&
        sunray_controller_->get_controller_state() ==
            uav_control::ControllerState::OFF) {
      return false;
    }
    return true;
  case SunrayState::LAND:
    if (should_use_px4_auto_land_locked()) {
      return false;
    }
    if (sunray_controller_ &&
        sunray_controller_->get_controller_state() ==
            uav_control::ControllerState::OFF) {
      return false;
    }
    return true;
  case SunrayState::OFF:
  default:
    return false;
  }
}

bool Sunray_StateMachine::should_use_px4_auto_land_locked() const {
  if (fsm_current_state_ != SunrayState::LAND) {
    return false;
  }
  if (!sunray_controller_) {
    return fsm_param_config_.land_type == 1;
  }
  return sunray_controller_->should_use_px4_auto_land();
}

bool Sunray_StateMachine::ensure_offboard_and_arm() {
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();

  if (!px4_state.connected) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] PX4 is not connected");
    return false;
  }

  if (!px4_set_mode_client_.isValid() || !px4_arming_client_.isValid()) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] mavros service clients are not ready");
    return false;
  }

  const ros::Time now = ros::Time::now();

  if (px4_state.flight_mode != px4_data_types::FlightMode::kOffboard) {
    if (px4_offboard_retry_state_.last_set_mode_req_time.isZero() ||
        (now - px4_offboard_retry_state_.last_set_mode_req_time).toSec() >=
            px4_offboard_retry_state_.set_mode_retry_interval_s) {
      mavros_msgs::SetMode mode_cmd;
      mode_cmd.request.custom_mode = "OFFBOARD";
      if (px4_set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("[SunrayFSM] OFFBOARD mode request sent");
      } else {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] OFFBOARD mode request failed");
      }
      px4_offboard_retry_state_.last_set_mode_req_time = now;
    }
    return false;
  }

  if (!px4_state.armed) {
    if (px4_offboard_retry_state_.last_arm_req_time.isZero() ||
        (now - px4_offboard_retry_state_.last_arm_req_time).toSec() >=
            px4_offboard_retry_state_.arm_retry_interval_s) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      if (px4_arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("[SunrayFSM] ARM request sent");
      } else {
        ROS_WARN_THROTTLE(1.0, "[SunrayFSM] ARM request failed");
      }
      px4_offboard_retry_state_.last_arm_req_time = now;
    }
    return false;
  }

  return true;
}

bool Sunray_StateMachine::ensure_disarm() {
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();

  if (!px4_state.connected) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] PX4 is not connected");
    return false;
  }

  if (!px4_arming_client_.isValid()) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] mavros arming client is not ready");
    return false;
  }

  if (!px4_state.armed) {
    return true;
  }

  const ros::Time now = ros::Time::now();
  if (px4_offboard_retry_state_.last_disarm_req_time.isZero() ||
      (now - px4_offboard_retry_state_.last_disarm_req_time).toSec() >=
          px4_offboard_retry_state_.disarm_retry_interval_s) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    if (px4_arming_client_.call(arm_cmd) && arm_cmd.response.success) {
      ROS_INFO("[SunrayFSM] DISARM request sent");
    } else {
      ROS_WARN_THROTTLE(1.0, "[SunrayFSM] DISARM request failed");
    }
    px4_offboard_retry_state_.last_disarm_req_time = now;
  }

  return false;
}

bool Sunray_StateMachine::ensure_auto_land_mode() {
  const px4_data_types::SystemState px4_state =
      px4_data_reader_.get_system_state();

  if (!px4_state.connected) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] PX4 is not connected");
    return false;
  }

  if (!px4_set_mode_client_.isValid()) {
    ROS_WARN_THROTTLE(1.0, "[SunrayFSM] mavros set_mode client is not ready");
    return false;
  }

  if (px4_state.flight_mode == px4_data_types::FlightMode::kAutoLand) {
    return true;
  }

  const ros::Time now = ros::Time::now();
  if (px4_offboard_retry_state_.last_set_mode_req_time.isZero() ||
      (now - px4_offboard_retry_state_.last_set_mode_req_time).toSec() >=
          px4_offboard_retry_state_.set_mode_retry_interval_s) {
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "AUTO.LAND";
    if (px4_set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
      ROS_INFO("[SunrayFSM] AUTO.LAND mode request sent");
    } else {
      ROS_WARN_THROTTLE(1.0, "[SunrayFSM] AUTO.LAND mode request failed");
    }
    px4_offboard_retry_state_.last_set_mode_req_time = now;
  }
  return false;
}
// 返回当前状态机的状态
SunrayState Sunray_StateMachine::get_current_state() const {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return fsm_current_state_;
}

double Sunray_StateMachine::get_supervisor_update_hz() const {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return std::max(1.0, fsm_param_config_.supervisor_update_hz);
}
// 将当前状态机的状态转换为字符串
const char *Sunray_StateMachine::to_string(SunrayState state) {
  switch (state) {
  case SunrayState::OFF:
    return "OFF";
  case SunrayState::TAKEOFF:
    return "TAKEOFF";
  case SunrayState::HOVER:
    return "HOVER";
  case SunrayState::RETURN:
    return "RETURN";
  case SunrayState::LAND:
    return "LAND";
  case SunrayState::EMERGENCY_LAND:
    return "EMERGENCY_LAND";
  case SunrayState::POSITION_CONTROL:
    return "POSITION_CONTROL";
  case SunrayState::VELOCITY_CONTROL:
    return "VELOCITY_CONTROL";
  case SunrayState::ATTITUDE_CONTROL:
    return "ATTITUDE_CONTROL";
  case SunrayState::COMPLEX_CONTROL:
    return "COMPLEX_CONTROL";
  case SunrayState::TRAJECTORY_CONTROL:
    return "TRAJECTORY_CONTROL";
  default:
    return "UNKNOWN_STATE";
  }
}
// 将当前状态机的事件转换为字符串
const char *Sunray_StateMachine::to_string(SunrayEvent event) {
  switch (event) {
  case SunrayEvent::TAKEOFF_REQUEST:
    return "TAKEOFF_REQUEST";
  case SunrayEvent::TAKEOFF_COMPLETED:
    return "TAKEOFF_COMPLETED";
  case SunrayEvent::LAND_REQUEST:
    return "LAND_REQUEST";
  case SunrayEvent::LAND_COMPLETED:
    return "LAND_COMPLETED";
  case SunrayEvent::EMERGENCY_REQUEST:
    return "EMERGENCY_REQUEST";
  case SunrayEvent::EMERGENCY_COMPLETED:
    return "EMERGENCY_COMPLETED";
  case SunrayEvent::RETURN_REQUEST:
    return "RETURN_REQUEST";
  case SunrayEvent::RETURN_COMPLETED:
    return "RETURN_COMPLETED";
  case SunrayEvent::WATCHDOG_ERROR:
    return "WATCHDOG_ERROR";
  case SunrayEvent::ENTER_POSITION_CONTROL:
    return "ENTER_POSITION_CONTROL";
  case SunrayEvent::ENTER_VELOCITY_CONTROL:
    return "ENTER_VELOCITY_CONTROL";
  case SunrayEvent::ENTER_ATTITUDE_CONTROL:
    return "ENTER_ATTITUDE_CONTROL";
  case SunrayEvent::ENTER_COMPLEX_CONTROL:
    return "ENTER_COMPLEX_CONTROL";
  case SunrayEvent::ENTER_TRAJECTORY_CONTROL:
    return "ENTER_TRAJECTORY_CONTROL";
  case SunrayEvent::POSITION_COMPLETED:
    return "POSITION_COMPLETED";
  case SunrayEvent::VELOCITY_COMPLETED:
    return "VELOCITY_COMPLETED";
  case SunrayEvent::ATTITUDE_COMPLETED:
    return "ATTITUDE_COMPLETED";
  case SunrayEvent::COMPLEX_COMPLETED:
    return "COMPLEX_COMPLETED";
  case SunrayEvent::TRAJECTORY_COMPLETED:
    return "TRAJECTORY_COMPLETED";
  default:
    return "UNKNOWN_EVENT";
  }
}

// 在起飞前/解锁前的安全检查
bool Sunray_StateMachine::check_health_preflight() {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return check_health_preflight_locked();
}

bool Sunray_StateMachine::check_health_preflight_locked() const {
  if (!sunray_controller_) {
    ROS_WARN(
        "[SunrayFSM] preflight check failed: controller is not registered");
    return false;
  }

  if (!validate_offstage_odometry_source_locked()) {
    ROS_WARN("[SunrayFSM] preflight check failed: odometry source is invalid "
             "in OFF stage");
    return false;
  }

  return true;
}
// 安全检查
bool Sunray_StateMachine::check_health() {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return check_health_locked();
}

bool Sunray_StateMachine::check_health_locked() const {
  // 当前为占位检查：
  // 1) 起飞前由 check_health_preflight() 执行完整检查
  // 2) 飞行中后续可扩展里程计稳定性、控制器输出饱和等检查。
  if (fsm_current_state_ == SunrayState::OFF) {
    return true;
  }
  // 返回控制器是否已注册
  return sunray_controller_ != nullptr;
}

// 起飞前里程计检查，检查里程计存在，有效，未超时
bool Sunray_StateMachine::validate_offstage_odometry_source() const {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return validate_offstage_odometry_source_locked();
}

bool Sunray_StateMachine::has_fresh_external_odom_locked(
    const ros::Time &now) const {
  if (!latest_external_odom_.isValid() ||
      latest_external_odom_.timestamp.isZero()) {
    return false;
  }

  const double age_s = (now - latest_external_odom_.timestamp).toSec();
  return age_s >= 0.0 && age_s <= fsm_param_config_.timeout_odom_s;
}

bool Sunray_StateMachine::validate_offstage_odometry_source_locked() const {
  if (!latest_external_odom_.isValid() ||
      latest_external_odom_.timestamp.isZero()) {
    ROS_WARN_THROTTLE(
        1.0, "[SunrayFSM] preflight blocked: no valid external odom injected");
    return false;
  }

  const double age_s =
      (ros::Time::now() - latest_external_odom_.timestamp).toSec();
  if (age_s < 0.0 || age_s > fsm_param_config_.timeout_odom_s) {
    ROS_WARN_THROTTLE(
        1.0,
        "[SunrayFSM] preflight blocked: external odom timeout age=%.3f s "
        "timeout=%.3f s",
        age_s, fsm_param_config_.timeout_odom_s);
    return false;
  }
  return true;
}

// 检查是否允许起飞
bool Sunray_StateMachine::can_takeoff() const {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return can_takeoff_locked();
}

bool Sunray_StateMachine::can_takeoff_locked() const {
  return validate_offstage_odometry_source_locked() &&
         static_cast<bool>(sunray_controller_);
}

// 里程计状态转移底层函数
bool Sunray_StateMachine::transition_to(SunrayState next_state) {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return transition_to_locked(next_state);
}

bool Sunray_StateMachine::apply_state_entry_action_locked(
    SunrayState next_state) {
  if (!sunray_controller_) {
    return next_state == SunrayState::OFF;
  }

  switch (next_state) {
  case SunrayState::TAKEOFF:
    px4_offboard_retry_state_.last_disarm_req_time = ros::Time(0);
    clear_active_control_source_locked();
    active_return_target_valid_ = false;
    land_after_return_pending_ = false;
    return_hover_start_time_ = ros::Time(0);
    return sunray_controller_->set_takeoff_mode(
        fsm_param_config_.takeoff_height_m,
        fsm_param_config_.takeoff_max_vel_mps);
  case SunrayState::LAND:
    px4_offboard_retry_state_.last_disarm_req_time = ros::Time(0);
    clear_active_control_source_locked();
    active_return_target_valid_ = false;
    land_after_return_pending_ = false;
    return_hover_start_time_ = ros::Time(0);
    (void)sunray_controller_->configure_landing(
        static_cast<uint8_t>(fsm_param_config_.land_type),
        fsm_param_config_.land_max_vel_mps);
    return sunray_controller_->set_land_mode();
  case SunrayState::EMERGENCY_LAND:
    px4_offboard_retry_state_.last_disarm_req_time = ros::Time(0);
    clear_active_control_source_locked();
    active_return_target_valid_ = false;
    land_after_return_pending_ = false;
    return_hover_start_time_ = ros::Time(0);
    return sunray_controller_->set_emergency_mode();
  case SunrayState::HOVER:
    clear_active_control_source_locked();
    active_return_target_valid_ = false;
    return sunray_controller_->set_hover_mode();
  case SunrayState::RETURN: {
    clear_active_control_source_locked();
    active_return_target_valid_ = false;
    land_after_return_pending_ = false;
    return_hover_start_time_ = ros::Time(0);
    uav_control::TrajectoryPoint return_target;
    if (!build_return_target_locked(&return_target)) {
      ROS_WARN("[SunrayFSM] RETURN entry failed: no valid home/target available");
      return false;
    }
    active_return_target_ = return_target;
    active_return_target_valid_ = true;
    (void)sunray_controller_->set_trajectory(return_target);
    return sunray_controller_->set_move_mode();
  }
  case SunrayState::POSITION_CONTROL:
  case SunrayState::VELOCITY_CONTROL:
  case SunrayState::ATTITUDE_CONTROL:
  case SunrayState::TRAJECTORY_CONTROL:
    active_return_target_valid_ = false;
    land_after_return_pending_ = false;
    return_hover_start_time_ = ros::Time(0);
    return sunray_controller_->set_move_mode();
  case SunrayState::COMPLEX_CONTROL:
    ROS_WARN("[SunrayFSM] COMPLEX_CONTROL is not wired to a controller path "
             "yet");
    return false;
  case SunrayState::OFF:
    px4_offboard_retry_state_.last_disarm_req_time = ros::Time(0);
    clear_active_control_source_locked();
    active_return_target_valid_ = false;
    land_after_return_pending_ = false;
    return_hover_start_time_ = ros::Time(0);
    return sunray_controller_->set_off_mode();
  default:
    return true;
  }
}

bool Sunray_StateMachine::build_return_target_locked(
    uav_control::TrajectoryPoint *target) {
  if (!target || !sunray_controller_) {
    return false;
  }

  const uav_control::UAVStateEstimate &current_state =
      sunray_controller_->get_current_state();
  if (!current_state.isValid()) {
    return false;
  }

  Eigen::Vector3d target_position = current_state.position;
  bool has_yaw = false;
  double target_yaw = 0.0;

  if (return_use_takeoff_homepoint_) {
    if (!sunray_controller_->has_home_position()) {
      return false;
    }
    const Eigen::Vector3d &home_position = sunray_controller_->get_home_position();
    const double min_return_altitude =
        home_position.z() + std::max(0.0, fsm_param_config_.takeoff_height_m);
    target_position.x() = home_position.x();
    target_position.y() = home_position.y();
    target_position.z() =
        std::max(current_state.position.z(), min_return_altitude);
  } else {
    target_position = Eigen::Vector3d(return_target_position_.x,
                                      return_target_position_.y,
                                      return_target_position_.z);
    if (!is_finite_vector3(target_position)) {
      return false;
    }
  }

  if (return_target_yaw_ctrl_) {
    if (!std::isfinite(return_target_yaw_)) {
      return false;
    }
    target_yaw = return_target_yaw_;
    has_yaw = true;
  }

  uav_control::TrajectoryPointReference target_ref;
  target_ref.clear_all();
  target_ref.set_position(target_position);
  if (has_yaw) {
    target_ref.set_yaw(target_yaw);
  }
  *target = target_ref.toRosMessage();

  if (return_use_takeoff_homepoint_) {
    const Eigen::Vector3d &home_position = sunray_controller_->get_home_position();
    ROS_INFO(
        "[SunrayFSM] build RETURN target using homepoint: "
        "current_odom=(%.3f, %.3f, %.3f) home=(%.3f, %.3f, %.3f) "
        "return_target=(%.3f, %.3f, %.3f)",
        current_state.position.x(), current_state.position.y(),
        current_state.position.z(), home_position.x(), home_position.y(),
        home_position.z(), target_position.x(), target_position.y(),
        target_position.z());
  } else {
    ROS_INFO(
        "[SunrayFSM] build RETURN target using explicit target: "
        "current_odom=(%.3f, %.3f, %.3f) return_target=(%.3f, %.3f, %.3f)",
        current_state.position.x(), current_state.position.y(),
        current_state.position.z(), target_position.x(), target_position.y(),
        target_position.z());
  }

  return true;
}

bool Sunray_StateMachine::is_return_target_reached_locked() const {
  const uav_control::TrajectoryPointReference active_return_target_ref(
      active_return_target_);
  if (!sunray_controller_ || !active_return_target_valid_ ||
      !active_return_target_ref.is_field_enabled(
          uav_control::TrajectoryPointReference::Field::POSITION)) {
    return false;
  }

  const uav_control::UAVStateEstimate &current_state =
      sunray_controller_->get_current_state();
  if (!current_state.isValid()) {
    return false;
  }

  const Eigen::Vector3d position_error =
      current_state.position - active_return_target_ref.position;
  return std::abs(position_error.x()) <= fsm_param_config_.error_tolerance_pos_x_m &&
         std::abs(position_error.y()) <= fsm_param_config_.error_tolerance_pos_y_m &&
         std::abs(position_error.z()) <= fsm_param_config_.error_tolerance_pos_z_m;
}

bool Sunray_StateMachine::is_position_target_reached_locked() const {
  if (!sunray_controller_) {
    return false;
  }

  const uav_control::UAVStateEstimate &current_state =
      sunray_controller_->get_current_state();
  if (!current_state.isValid()) {
    return false;
  }

  const uav_control::TrajectoryPointReference target_ref(
      sunray_controller_->get_trajectory_reference());
  if (!target_ref.is_field_enabled(
          uav_control::TrajectoryPointReference::Field::POSITION)) {
    return false;
  }

  const Eigen::Vector3d position_error = current_state.position - target_ref.position;
  return std::abs(position_error.x()) <= fsm_param_config_.error_tolerance_pos_x_m &&
         std::abs(position_error.y()) <= fsm_param_config_.error_tolerance_pos_y_m &&
         std::abs(position_error.z()) <= fsm_param_config_.error_tolerance_pos_z_m;
}

bool Sunray_StateMachine::transition_to_locked(SunrayState next_state) {
  if (fsm_current_state_ == next_state) {
    return true;
  }

  if (fsm_current_state_ == SunrayState::OFF &&
      next_state == SunrayState::TAKEOFF && !check_health_preflight_locked()) {
    ROS_WARN("[SunrayFSM] transition blocked: OFF -> TAKEOFF preflight check "
             "failed");
    return false;
  }

  if (!apply_state_entry_action_locked(next_state)) {
    ROS_WARN("[SunrayFSM] transition blocked: %s -> %s entry action failed",
             to_string(fsm_current_state_), to_string(next_state));
    return false;
  }

  ROS_INFO("[SunrayFSM] transition: %s -> %s", to_string(fsm_current_state_),
           to_string(next_state));
  fsm_current_state_ = next_state;
  return true;
}
// 获取当前控制器指针
std::shared_ptr<uav_control::Base_Controller>
Sunray_StateMachine::get_controller() const {
  std::lock_guard<std::mutex> lock(fsm_mutex_);
  return sunray_controller_;
}

void Sunray_StateMachine::publish_fsm_state() {
  const ros::Publisher fsm_state_pub = fsm_state_pub_;
  if (!fsm_state_pub) {
    return;
  }

  std_msgs::UInt8 msg;
  {
    std::lock_guard<std::mutex> lock(fsm_mutex_);
    msg.data = static_cast<uint8_t>(fsm_current_state_);
  }
  fsm_state_pub.publish(msg);
}

} // namespace sunray_fsm
