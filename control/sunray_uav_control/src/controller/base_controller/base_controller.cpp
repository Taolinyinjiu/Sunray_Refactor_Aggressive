#include "controller/base_controller/base_controller.hpp"

#include <algorithm>
#include <cmath>

namespace uav_control {

/**
 * @brief 切换控制器到起飞模式。
 *
 * @details
 * 仅允许在 `OFF` 状态下调用
 * 该函数会锁存地面参考高度，生成起飞目标点（当前位置 +`relative_takeoff_height_`），
 * 并重置起飞过程上下文后切换到 `TAKEOFF` 状态。
        @param relative_takeoff_height 相对起飞高度
        @param max_takeoff_velocity 起飞过程中的最大速度
 * @return true 切换成功；false 前置条件不满足。
 */
bool Base_Controller::set_takeoff_mode(double relative_takeoff_height,
                                       double max_takeoff_velocity) {
  // 如果控制器当前状态不为OFF状态，则返回false
  if (controller_state_ != ControllerState::OFF) {
    return false;
  }
  reset_trajectory_tracking();
  trajectory_completed_ = false;
  
		// 刷新home位置
		home_position_ = uav_current_state_.position;
  home_position_initialized_ = true;

		// 更新起飞最大速度
  takeoff_max_velocity_ = max_takeoff_velocity;

  // 进入起飞流程时刷新地面参考高度，供后续降落目标使用。
  ground_reference_z_ = uav_current_state_.position.z();
  ground_reference_initialized_ = true;

  // 以当前位置信息作为起飞参考，并叠加相对起飞高度。
  takeoff_expect_position_ = uav_current_state_.position;
  takeoff_expect_position_.z() += relative_takeoff_height;

  // 进入 TAKEOFF 前重置起飞过程上下文。
  takeoff_initialized_ = false;
  takeoff_holdstart_time_ = ros::Time(0);
  takeoff_holdkeep_time_ = ros::Time(0);
  controller_state_ = ControllerState::TAKEOFF;
  return true;
}

/**
 * @brief 切换控制器到降落模式。
 *
 * @details
 * 若当前已在 `LAND` 状态则视为幂等成功。
 * 否则要求当前状态不是 `OFF/UNDEFINED`
 * 降落目标点以当前位置为基准，若已锁存地面高度则使用
 * `min(current_z, ground_reference_z_)` 防止先上升后下降。
 *
 * @return true 切换成功；false 前置条件不满足。
 */
bool Base_Controller::set_land_mode() {
  // 检查当前是否已经处于LAND状态，是则返回true
  if (controller_state_ == ControllerState::LAND) {
    return true;
  }
  // 检查当前是否处于UNDEFINE或者OFF状态,以及里程计是否有效
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }
  reset_trajectory_tracking();
  trajectory_completed_ = false;

  // 以当前位置作为降落参考；降落高度优先使用锁存的地面参考高度。
  land_expect_position_ = uav_current_state_.position;
  // 如果地面高度已经初始化(这个应当在起飞阶段保证)
  if (ground_reference_initialized_) {
    // 在当前位置的z轴数据和锁存的地面参考z轴高度进行对比,取小者
    land_expect_position_.z() =
        std::min(uav_current_state_.position.z(), ground_reference_z_);
  } else {
    land_expect_position_.z() = uav_current_state_.position.z();
  }

  ROS_INFO(
      "[Base_Controller] enter LAND: current_odom=(%.3f, %.3f, %.3f) "
      "ground_z=%.3f land_expect=(%.3f, %.3f, %.3f) land_type=%u",
      uav_current_state_.position.x(), uav_current_state_.position.y(),
      uav_current_state_.position.z(), ground_reference_z_,
      land_expect_position_.x(), land_expect_position_.y(),
      land_expect_position_.z(), static_cast<unsigned>(land_type_));

  // 降落类型仅保留“按配置/请求显式选择”的语义，不再因为高度参考退化而
  // 自动强制切到 AUTO.LAND；这样可以稳定复用代码侧末段降落逻辑。
  land_initialized_ = false;
  land_holdstart_time_ = ros::Time(0);
  land_holdkeep_time_ = ros::Time(0);
  land_low_velocity_start_time_ = ros::Time(0);
  land_touchdown_detected_time_ = ros::Time(0);
  controller_state_ = ControllerState::LAND;
  return true;
}

/**
 * @brief
 * EMERGENCY_LAND作为一个实验性的模式，暂时咩有太好的方式来实现
 * @details
 * `OFF` 和 `UNDEFINED` 状态下拒绝切换，其余状态直接进入
 * `EMERGENCY_LAND`。
 *
 * @return true 切换成功；false 当前状态不允许。
 */
bool Base_Controller::set_emergency_mode() {
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }
  reset_trajectory_tracking();
  trajectory_completed_ = false;
  controller_state_ = ControllerState::EMERGENCY_LAND;
  return true;
}

void Base_Controller::reset_trajectory_tracking() {
  trajectory_tracking_enabled_ = false;
  trajectory_buffer_.clear();
}

bool Base_Controller::update_trajectory_reference_from_buffer(
    const ros::Time &now) {
  if (!trajectory_tracking_enabled_) {
    return false;
  }

  trajectory_buffer_.update_active(now);
  const TrajectorySample sample = trajectory_buffer_.sample(now);
  if (!sample.valid) {
    return false;
  }

  trajectory_ = sample.point;
  if (sample.reached_end) {
    trajectory_tracking_enabled_ = false;
    trajectory_completed_ = true;
    trajectory_buffer_.mark_completed();
  }
  return true;
}

bool Base_Controller::set_off_mode() {
  if (!uav_current_state_.isValid()) {
    return false;
  }

  reset_trajectory_tracking();
  trajectory_completed_ = false;
  TrajectoryPointReference trajectory_ref;
  trajectory_ref.clear_all();
  trajectory_ref.set_position(uav_current_state_.position);
  trajectory_ = trajectory_ref.toRosMessage();
  controller_state_ = ControllerState::OFF;
  return true;
}

bool Base_Controller::configure_landing(uint8_t land_type,
                                        double land_max_velocity) {
  land_type_ = (land_type == 0U) ? 0U : 1U;
  land_max_velocity_ = (land_max_velocity > 0.0) ? -land_max_velocity
                                                 : land_max_velocity;
  if (std::abs(land_max_velocity_) < 1e-6) {
    land_max_velocity_ = -0.5;
  }
  return true;
}

bool Base_Controller::set_hover_mode() {
  if (!uav_current_state_.isValid()) {
    return false;
  }

  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }

  reset_trajectory_tracking();
  trajectory_completed_ = false;
  TrajectoryPointReference trajectory_ref;
  trajectory_ref.clear_all();
  trajectory_ref.set_position(uav_current_state_.position);
  trajectory_ = trajectory_ref.toRosMessage();
  controller_state_ = ControllerState::HOVER;
  return true;
}

bool Base_Controller::set_move_mode() {
  if (controller_state_ == ControllerState::OFF ||
      controller_state_ == ControllerState::UNDEFINED) {
    return false;
  }

  if (trajectory_tracking_enabled_ && trajectory_buffer_.has_reference()) {
    controller_state_ = ControllerState::MOVE;
    return true;
  }

  const TrajectoryPointReference trajectory_ref(trajectory_);
  if (trajectory_ref.valid_mask ==
      static_cast<uint32_t>(TrajectoryPointReference::Field::UNDEFINED)) {
    return false;
  }

  controller_state_ = ControllerState::MOVE;
  return true;
}

/**
 * @brief 更新 PX4 解锁状态缓存。
 *
 * @param arm_state true 表示已解锁；false 表示未解锁。
 * @return true 始终返回成功。
 */
bool Base_Controller::set_px4_arm_state(bool arm_state) {
  px4_arm_state_ = arm_state;
  return true;
}

/**
 * @brief 更新当前状态估计（里程计）缓存。
 *
 * @param current_state 输入状态估计。
 * @return true 始终返回成功。
 *
 * @details
 * 首次收到有效状态时会自动锁存 `ground_reference_z_`，供后续降落逻辑使用。
 */
bool Base_Controller::set_current_odom(const UAVStateEstimate &current_state) {
  uav_current_state_ = current_state;
  if (!ground_reference_initialized_ && uav_current_state_.isValid()) {
    ground_reference_z_ = uav_current_state_.position.z();
    ground_reference_initialized_ = true;
  }
	controller_ready_ = true;
  return true;
}

/**
 * @brief 获取当前缓存的状态估计。
 *
 * @return 当前状态估计对象的常量引用。
 */
const UAVStateEstimate &Base_Controller::get_current_state() const {
  return uav_current_state_;
}

/**
 * @brief 更新 PX4 姿态四元数缓存。
 *
 * @param imu_msg 输入 IMU 消息（读取其 `orientation` 字段）。
 * @return true 始终返回成功。
 */
bool Base_Controller::set_px4_attitude(const sensor_msgs::Imu &imu_msg) {
  px4_attitude_.x() = imu_msg.orientation.x;
  px4_attitude_.y() = imu_msg.orientation.y;
  px4_attitude_.z() = imu_msg.orientation.z;
  px4_attitude_.w() = imu_msg.orientation.w;
  return true;
}

bool Base_Controller::set_px4_land_status(const bool land_status){
	if(land_status == false)
	{
		px4_land_status_ = false; // 置位
		// 清空时间参数
		land_holdstart_time_ = ros::Time(0);
		land_holdkeep_time_ = ros::Time(0);
		// 结束
		return true;
	}
	// 进入到这里说明设置的值为true 
	if(px4_land_status_ == false){
		px4_land_status_ = true; // 首先，置位
		land_holdstart_time_ = ros::Time::now(); // 设置检测落地接触时间为当前时间
	}
	return true;
}

/**
 * @brief 更新控制参考轨迹点。
 *
 * @param trajectory 输入轨迹点。
 * @return true 始终返回成功。
 *
 * @details
 * 当 `valid_mask` 为 `UNDEFINED` 时，会按非零字段推断有效通道，
 * 用于兼容未显式设置有效位的旧调用路径。
 */
bool Base_Controller::set_trajectory(
    const uav_control::TrajectoryPoint &trajectory) {
  reset_trajectory_tracking();
  trajectory_completed_ = false;
  trajectory_ = trajectory;
  TrajectoryPointReference trajectory_ref(trajectory_);
  if (trajectory_ref.valid_mask ==
      static_cast<uint32_t>(TrajectoryPointReference::Field::UNDEFINED)) {
    trajectory_ref.infer_valid_mask_from_nonzero();
    trajectory_ = trajectory_ref.toRosMessage();
  }
  return true;
}

bool Base_Controller::set_trajectory(const uav_control::Trajectory &trajectory) {
  if (trajectory.points.empty()) {
    return false;
  }

  if (trajectory.points.size() == 1U) {
    return set_trajectory(trajectory.points.front());
  }

  reset_trajectory_tracking();
  trajectory_completed_ = false;

  const ros::Time now = ros::Time::now();
  const uint32_t fallback_id = next_trajectory_id_++;
  if (!trajectory_buffer_.push_pending(trajectory, now, fallback_id)) {
    return false;
  }

  // 为将来时生效的轨迹预置一个安全保持参考，避免沿用上一条控制命令。
  if (uav_current_state_.isValid()) {
    TrajectoryPointReference hold_ref;
    hold_ref.clear_all();
    hold_ref.set_position(uav_current_state_.position);
    trajectory_ = hold_ref.toRosMessage();
  } else {
    TrajectoryPointReference first_ref(trajectory.points.front());
    if (first_ref.valid_mask ==
        static_cast<uint32_t>(TrajectoryPointReference::Field::UNDEFINED)) {
      first_ref.infer_valid_mask_from_nonzero();
    }
    trajectory_ = first_ref.toRosMessage();
  }

  trajectory_tracking_enabled_ = true;
  (void)update_trajectory_reference_from_buffer(now);
  return true;
}

/**
 * @brief 获取控制器内部状态机当前状态。
 *
 * @return 当前 `ControllerState`。
 */
ControllerState Base_Controller::get_controller_state() const {
  return controller_state_;
}

const uav_control::TrajectoryPoint &Base_Controller::get_trajectory_reference()
    const {
  return trajectory_;
}

bool Base_Controller::has_home_position() const {
  return home_position_initialized_;
}

bool Base_Controller::update_home_position(Eigen::Vector3d position) const {
  if (!position.allFinite()) {
    return false;
  }

  Base_Controller *self = const_cast<Base_Controller *>(this);
  self->home_position_ = position;
  self->home_position_initialized_ = true;
  return true;
}

bool Base_Controller::update_home_position(Eigen::Vector3d position,
                                           double yaw) const {
  if (!std::isfinite(yaw)) {
    return false;
  }

  return update_home_position(position);
}

const Eigen::Vector3d &Base_Controller::get_home_position() const {
  return home_position_;
}

bool Base_Controller::should_use_px4_auto_land() const {
  return land_type_ == 1U;
}

/**
 * @brief 默认起飞完成判定。
 *
 * @return true 当前状态为 `HOVER`；否则 false。
 */
bool Base_Controller::is_takeoff_completed() const {
  if (controller_state_ == ControllerState::HOVER)
    return true;
  return false;
}

/**
 * @brief 默认降落完成判定。
 *
 * @return true 当前状态为 `OFF`；否则 false。
 */
bool Base_Controller::is_land_completed() const {
  if (controller_state_ == ControllerState::OFF)
    return true;
  return false;
}

/**
 * @brief 默认紧急降落完成判定。
 *
 * @return true 当前状态为 `OFF`；否则 false。
 */
bool Base_Controller::is_emergency_completed() const {
  if (controller_state_ == ControllerState::OFF)
    return true;
  return false;
}

bool Base_Controller::is_trajectory_completed() const {
  return trajectory_completed_;
}

/**
 * @brief 控制器健康检查（基础实现）。
 *
 * @return true 参数已加载且当前状态估计有效；否则 false。
 */
bool Base_Controller::is_ready() const {
  return controller_ready_ && uav_current_state_.isValid();
}

} // namespace uav_control
