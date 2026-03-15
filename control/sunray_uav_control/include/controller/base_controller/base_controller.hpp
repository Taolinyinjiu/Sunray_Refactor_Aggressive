/**
 * @class Base_Controller
 * @brief 无人机控制器抽象基类。
 *
 * @details
 * 该类定义了 Sunray 控制链路中“控制器组件”的统一接口契约，职责边界如下：
 * - 由上层状态机/任务层进行任务决策与模式切换；
 * - 控制器接收当前状态、目标参考与模式命令，并在 `update()` 中输出控制量；
 * - 具体控制律（位置环、速度环、姿态环、轨迹跟踪等）由子类实现。
 *
 * 典型调用顺序：
 * 1. `load_param()`：加载控制参数；
 * 2. 周期性注入状态：`set_current_odom()` / `set_px4_attitude()`；
 * 3. 按任务切换模式：`set_takeoff_mode()` / `set_land_mode()` /
 * `set_emergency_mode()`；
 * 4. 周期性检查控制器状态：`get_controller_state()`
 * 5. 周期调用 `update()` 获取控制输出。
 *
 * @note 该基类侧重“接口一致性与公共状态容器”，不负责高层任务规划。
 */

#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_control/Trajectory.h>

#include <Eigen/Dense>

#include "control_data_types/control_data_types.h"
#include "control_msg_types/trajectory_point.hpp"
#include "control_data_types/uav_state_estimate.hpp"
#include "trajectory_buffer/trajectory_buffer.h"

namespace uav_control {

class Base_Controller {
public:
  Base_Controller() = default;
  virtual ~Base_Controller() {} // 必须为虚析构

  /**
   * @brief 以相对高度方式切换到起飞模式。
   * @param relative_takeoff_height 相对当前高度的起飞增量，单位 m。
   * @param max_takeoff_velocity 起飞阶段最大速度，单位 m/s
   * @return true 切换成功；false 参数非法或状态不允许。
   * @note 该参数语义为“相对高度”，不是绝对世界高度。
   */
  virtual bool set_takeoff_mode(double relative_takeoff_height,
                                double max_takeoff_velocity);

  /**
   * @brief 设置飞控解锁状态。
   * @param arm_state true 表示已解锁；false 表示未解锁。
   * @return true 设置成功。
   */
  virtual bool set_px4_arm_state(bool arm_state);

  /**
   * @brief 切换到降落模式。
   * @return true 切换成功；false 切换失败。
   * @details 默认语义为“进入降落控制阶段”，降落目标由任务层与目标输入约定决定。
   */
  virtual bool set_land_mode(void);

  /**
   * @brief 切换到紧急降落模式。
   * @return true 切换成功；false 切换失败。
   * @details 紧急降落通常用于健康检查失败、链路异常等紧急场景。
   */
  virtual bool set_emergency_mode(void);

  /**
   * @brief 切换到地面待机模式。
   * @return true 切换成功；false 当前状态或输入不允许。
   */
  virtual bool set_off_mode(void);

  /**
   * @brief 配置降落策略参数。
   * @param land_type 降落类型，0 为代码控制降落，1 为 PX4 AUTO.LAND。
   * @param land_max_velocity 降落过程最大速度，单位 m/s。
   * @return true 参数写入成功。
   */
  virtual bool configure_landing(uint8_t land_type, double land_max_velocity);

  /**
   * @brief 切换到悬停保持模式。
   * @return true 切换成功；false 当前状态或输入不允许。
   * @details 默认实现会锁存当前位置为保持点，并切换到 `HOVER`。
   */
  virtual bool set_hover_mode(void);

  /**
   * @brief 切换到轨迹/运动控制模式。
   * @return true 切换成功；false 当前状态或参考输入不允许。
   * @details 默认实现要求已存在有效轨迹参考，并切换到 `MOVE`。
   */
  virtual bool set_move_mode(void);

  /**
   * @brief 更新无人机当前状态估计（里程计侧）。
   * @param current_state 当前状态估计。
   * @return true 写入成功。
   */
  virtual bool set_current_odom(const UAVStateEstimate &current_state);

  /**
   * @brief 获取无人机当前状态估计（里程计侧）。
   * @return 当前状态估计只读引用。
   */
  virtual const UAVStateEstimate &get_current_state() const;

  /**
   * @brief 更新 PX4 侧姿态输入（IMU）。
   * @param imu_msg IMU 消息。
   * @return true 写入成功。
   * @note 该接口与 `set_current_odom()` 的数据源可不同，保留解耦设计。
   */
  virtual bool set_px4_attitude(const sensor_msgs::Imu &imu_msg);

  /**
   * @brief 更新 PX4 降落检测器状态
   * @param land_status
   * @return true 写入成功。
   */
  virtual bool set_px4_land_status(const bool land_status);

  /**
   * @brief 设置控制参考输入（轨迹点）。
   * @param trajectory 期望轨迹点（位置/速度/加速度/yaw 等）。
   * @return true 写入成功。
   */
  virtual bool set_trajectory(const uav_control::TrajectoryPoint &trajectory);

  /**
   * @brief 设置多点轨迹参考输入。
   * @param trajectory 轨迹消息，支持单点与多点。
   * @return true 写入成功；false 轨迹非法。
   */
  virtual bool set_trajectory(const uav_control::Trajectory &trajectory);

  /**
   * @brief 获取控制器内部状态机状态。
   * @return 当前控制器状态。
   */
  virtual ControllerState get_controller_state() const;

  /**
   * @brief 获取当前锁存的参考轨迹点。
   * @return 当前参考轨迹点。
   */
  virtual const uav_control::TrajectoryPoint &get_trajectory_reference() const;

  /**
   * @brief 是否已经锁存起飞 home 点。
   * @return true 已锁存；false 尚未锁存。
   */
  virtual bool has_home_position() const;

	/**
   * @brief 更新home点，用于状态机返航接口
   * @return true 修改成功；false 修改失败
   */
	virtual bool update_home_position(Eigen::Vector3d position) const ;
	virtual bool update_home_position(Eigen::Vector3d position,double yaw) const ;

  /**
   * @brief 获取锁存的起飞 home 点。
   * @return home 点位置。
   */
  virtual const Eigen::Vector3d &get_home_position() const;

  /**
   * @brief 当前降落策略是否要求切 PX4 AUTO.LAND。
   * @return true 需要 PX4 自动降落；false 使用代码控制降落。
   */
  virtual bool should_use_px4_auto_land() const;

  /**
   * @brief 判定起飞任务是否完成。
   * @return true 表示完成；false 表示未完成。
   * @note 建议子类按控制器特性（高度、速度、稳定时间）重写。
   */
  virtual bool is_takeoff_completed() const;

  /**
   * @brief 判定降落任务是否完成。
   * @return true 表示完成；false 表示未完成。
   * @note 建议子类按控制器特性（高度接地、速度阈值）重写。
   */
  virtual bool is_land_completed() const;

  /**
   * @brief 控制器侧是否已经锁存“触地”。
   * @return true 表示控制器根据自身判据认为已触地；false 表示尚未触地。
   * @note 该接口用于在 PX4 landed_state 缺失时提供收尾兜底，不等价于最终任务完成。
   */
  virtual bool is_touchdown_detected() const;

  /**
   * @brief 获取控制器侧触地锁存已持续的时长。
   * @param now 当前时间。
   * @return 若尚未触地则返回 0。
   */
  virtual double get_touchdown_elapsed_s(const ros::Time &now) const;

  /**
   * @brief 判定紧急降落任务是否完成。
   * @return true 表示完成；false 表示未完成。
   */
  virtual bool is_emergency_completed() const;

  /**
   * @brief 判定多点轨迹是否已经执行完成。
   * @return true 表示当前活动轨迹已到末端；false 表示尚未完成。
   */
  virtual bool is_trajectory_completed() const;

  /**
   * @brief 控制器通用就绪状态检查。
   * @return true 控制器已就绪；false 控制器还未就绪。
   * @details 建议覆盖检查项包括参数合法性、输入时效性、状态有效性等。
   */
  virtual bool is_ready() const;

  /**
   * @brief 控制律核心更新函数。
   * @return 控制输出（由 output mask 声明有效通道）。
   * @note 该函数应满足“可周期调用、无副作用泄漏、对无效输入可安全退化”。
   */
  virtual ControllerOutput update(void) = 0;

protected:
  void reset_trajectory_tracking();
  bool update_trajectory_reference_from_buffer(const ros::Time &now);

  /** ---------------基本参数----------------- */

  /** @brief 控制器就绪状态。 */
  bool controller_ready_ = false;

  /** @brief PX4 解锁状态。 */
  bool px4_arm_state_ = false;

  /** @brief 误差容限数组，通常为 `{x_tol, y_tol, z_tol}`。 */
  Eigen::Vector3d error_tolerance_ = Eigen::Vector3d(0.2,0.2,0.2);

  /** @brief 三轴最大速度参数，通常为`x_vel,y_vel,z_vel` */
  Eigen::Vector3d velocity_max_ = Eigen::Vector3d(2.0,2.0,2.0);

  /** ---------------地面参数----------------- */

  /** @brief 地面参考高度（m），通常由首次有效里程计/起飞时刻锁存。 */
  double ground_reference_z_ = 0.0;

  /** @brief 地面参考高度是否已初始化。 */
  bool ground_reference_initialized_ = false;

  /** @brief 起飞 home 点是否已锁存。 */
  bool home_position_initialized_ = false;

  /** ---------------起飞参数----------------- */
  /** @brief 起飞状态上下文 */
  bool takeoff_initialized_ = false;
	
	/** @brief 起飞时的位置 */
	Eigen::Vector3d home_position_;

  /** @brief 起飞期望位置（m）。 */
  Eigen::Vector3d takeoff_expect_position_;

  /** @brief 起飞过程中最大速度 */
  double takeoff_max_velocity_ = 0.5;

  /** @brief 计算出来的理论运动时间，小于 @param takeoff_singlecurve_limit_time
   * 则切换为多段曲线拼接式的起飞模式 */
  double takeoff_singlecurve_limit_time_ = 3.0;
	
	double takeoff_singlecurve_time_ = 0.0;

  /** @brief 起飞完成判定所需保持时间（s）。 */
  double takeoff_success_time_ = 3.0;
	
	/** @brief 起飞开始时间戳。 */
  ros::Time takeoff_start_time_ = ros::Time(0);

  /** @brief 起飞稳定区间开始时间戳。 */
  ros::Time takeoff_holdstart_time_ = ros::Time(0);

  /** @brief 起飞稳定区间最近保持时间戳。 */
  ros::Time takeoff_holdkeep_time_ = ros::Time(0);

  /** ---------------降落参数----------------- */
  /** @brief 降落类型 0:基于五次项曲线实现的降落 1:px4.auto_land */
  uint8_t land_type_ = 0;
	
	/*** @brief px4传入的降落检测状态 */	
	bool px4_land_status_ = false;

  /** @brief 降落状态上下文 */
  bool land_initialized_ = false;

  /** @brief 降落期望参考位置（m）。 */
  Eigen::Vector3d land_expect_position_;

  /** @brief 降落过程中最大速度 */
  double land_max_velocity_ = -0.5;

  /** @brief 限制运动时间 */
  double land_singlecurve_limit_time_ = 3.0;
	/*** @brief 计算出来的理论运动时间 */
	double land_singlecurve_time_ = 0.0;
	
  /** @brief 降落完成判定所需保持时间（s）。 */
  double land_success_time_ = 3.0;

			/** @brief 降落开始时间戳。 */
  ros::Time land_start_time_ = ros::Time(0);

  /** @brief 降落稳定区间开始时间戳。 */
  ros::Time land_holdstart_time_ = ros::Time(0);

  /** @brief 降落稳定区间最近保持时间戳。 */
  ros::Time land_holdkeep_time_ = ros::Time(0);

  /** @brief 降落阶段锁存的偏航角（rad）。 */
  double land_yaw_ = 0.0;

  /** @brief 末段降落的 XY 速度控制比例系数。 */
  double land_xy_kp_ = 1.0;

  /** @brief 末段降落的 XY 最大修正速度（m/s）。 */
  double land_max_velocity_xy_mps_ = 0.5;

  /** @brief 低速触地判定的速度阈值（m/s）。 */
  double land_touchdown_velocity_threshold_mps_ = 0.1;

  /** @brief 低速触地判定所需持续时间（s）。 */
  double land_touchdown_velocity_hold_time_s_ = 1.0;

  /** @brief 启用低速触地判定的近地高度阈值（m）。 */
  double land_touchdown_height_threshold_m_ = 0.15;

  /** @brief 检测到触地后继续下压的速度（m/s）。 */
  double land_touchdown_downpress_speed_mps_ = 0.2;

  /** @brief 检测到触地后继续下压的持续时间（s）。 */
  double land_touchdown_downpress_time_s_ = 1.0;

  /** @brief “近地低速”开始时间戳。 */
  ros::Time land_low_velocity_start_time_ = ros::Time(0);

  /** @brief 首次判定触地的时间戳。 */
  ros::Time land_touchdown_detected_time_ = ros::Time(0);

  /** ---------------运动参数----------------- */
  /** @brief 当前控制参考轨迹点。 */
  uav_control::TrajectoryPoint trajectory_;
  TrajectoryBufferManager trajectory_buffer_{};
  bool trajectory_tracking_enabled_{false};
  bool trajectory_completed_{false};
  uint32_t next_trajectory_id_{1U};

  /** @brief 控制器内部状态机当前状态。 */
  ControllerState controller_state_ = ControllerState::UNDEFINED;

  /** @brief 当前状态估计（主输入状态）。 */
  UAVStateEstimate uav_current_state_;

  /** @brief PX4 姿态输入（可与里程计来源解耦）。 */
  Eigen::Quaterniond px4_attitude_ = Eigen::Quaterniond::Identity();
};

} // namespace uav_control
