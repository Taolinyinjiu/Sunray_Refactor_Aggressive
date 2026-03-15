/***
        @brief 先构造辅助类
        辅助类的目的是向开发者提供一个友好的,高宽容度的Sunray无人机控制接口,Helper主要与Surnay_FSM进行通信,获取Sunray_FSM的状态,无人机的状态,以及调用相关的运动接口
        */

#pragma once

#include "sunray_statemachine/sunray_statemachine_datatypes.h"
#include "control_data_types/control_data_types.h"
#include "control_msg_types/trajectory_point.hpp"
#include <control_data_types/uav_state_estimate.hpp>
#include <memory>
#include <mutex>
#include <px4_bridge/px4_data_reader.h>
#include <px4_bridge/px4_param_manager.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt8.h>
#include <uav_control/Trajectory.h>
#include <uav_control/TrajectoryPoint.h>
#include <string>
#include <utility>
#include <vector>

/***
        @note
   所有提供了阻塞接口的函数，底层都使用服务进行实现，理由是，我们在设计的过程中，认为凡是提供了阻塞接口的函数，天然就不具有高频控制的理念或者期望，因此使用话题是一种浪费或者不必要
*/

class Sunray_Helper {
public:
  // 构造函数
  Sunray_Helper(ros::NodeHandle &nh_);
  // 析构函数
  ~Sunray_Helper() = default;
  // async 异步，非阻塞,使用话题的形式发布
  // block(spin) 同步，阻塞，使用服务的方式实现，没有服务接口的，使用话题回调
  // 状态检验实现
  // -------------------------基本控制接口--------------------------------
  // 触发起飞
  bool takeoff_async();
  bool takeoff_block();
  /** @param  relative_takeoff_height
     传入起飞高度，当传入零或者负值时表示使用参数文件中的起飞高度
                @param 	max_takeoff_velocity
     传入起飞过程中的最大速度，当传入零或者负值时表示使用参数文件中的最大起飞速度
        */
  bool takeoff_async(double relative_takeoff_height,
                     double max_takeoff_velocity);
  bool takeoff_block(double relative_takeoff_height,
                     double max_takeoff_velocity);

  // 触发降落
  bool land_async();
  bool land_block();
  /** @param  land_type
     传入降落类型，负值或者零表示使用代码降落，正值表示使用px4.auto_land
                @param 	land_max_velocity
     传入降落过程中的最大速度，当传入零或者负值时表示使用参数文件中的最大降落速度
        */
  bool land_async(int land_type, double land_max_velocity);
  bool land_block(int land_type, double land_max_velocity);
  // 触发返航
  bool return_async();
  bool return_block();
  bool return_async(Eigen::Vector3d target_position);
  bool return_block(Eigen::Vector3d target_position);
  bool return_async(Eigen::Vector3d target_position, double target_yaw);
  bool return_block(Eigen::Vector3d target_position, double target_yaw);
  bool return_async(Eigen::Vector3d target_position, double target_yaw,
                    int land_type);
  bool return_block(Eigen::Vector3d target_position, double target_yaw,
                    int land_type);
  bool return_async(Eigen::Vector3d target_position, double target_yaw,
                    int land_type, double land_max_velocity);
  bool return_block(Eigen::Vector3d target_position, double target_yaw,
                    int land_type, double land_max_velocity);

  // 触发位置控制
  bool set_position_async(Eigen::Vector3d target_position);
  bool set_position_block(Eigen::Vector3d target_position);
  // 多点位置异步接口当前通过 trajectory 控制链路实现。
  bool
  set_position_list_async(std::vector<Eigen::Vector3d> target_position_list);
  // 多点位置阻塞接口按顺序逐点执行，前一点到达后再发下一点。
  bool
  set_position_list_block(std::vector<Eigen::Vector3d> target_position_list);

  // 带有yaw角的位置控制
  bool set_position_async(Eigen::Vector3d target_position, double target_yaw);
  bool set_position_block(Eigen::Vector3d target_position, double target_yaw);
  bool set_position_async(Eigen::Vector3d target_position, double target_yaw,
                          double target_yaw_rate);
  bool set_position_block(Eigen::Vector3d target_position, double target_yaw,
                          double target_yaw_rate);
  // 多点位置+yaw 异步接口当前通过 trajectory 控制链路实现。
  bool set_position_list_async(
      std::vector<std::pair<Eigen::Vector3d, double>> target_position_list);
  // 多点位置+yaw 阻塞接口按顺序逐点执行，前一点到达后再发下一点。
  bool set_position_list_block(
      std::vector<std::pair<Eigen::Vector3d, double>> target_position_list);
  // 该接口内部使用 trajectory 控制链路传递 heading_rate；若底层控制器未消费，
  // 则 yaw_rate 只会作为参考字段缓存，不保证严格生效。
  bool set_position_list_async(
      std::vector<std::pair<Eigen::Vector3d, double>> target_position_list,
      double target_yaw_rate);
  // 阻塞版本仍按顺序逐点执行；由于 position service 本身不支持 yaw_rate，
  // 这里只保证位置与 yaw 目标，不保证严格的 yaw_rate 约束。
  bool set_position_list_block(
      std::vector<std::pair<Eigen::Vector3d, double>> target_position_list,
      double target_yaw_rate);
  // 带有速度限制的位置控制
  bool set_position_velocity_async(Eigen::Vector3d target_position,
                                   double target_velocity);
  bool set_position_velocity_block(Eigen::Vector3d target_position,
                                   double target_velocity);
  // 多点位置+速度异步接口当前通过 trajectory 控制链路实现。
  bool set_position_velocity_list_async(
      std::vector<std::pair<Eigen::Vector3d, double>> target_position_list);
  // 多点位置+速度阻塞接口按顺序逐点执行。
  bool set_position_velocity_list_block(
      std::vector<std::pair<Eigen::Vector3d, double>> target_position_list);
  // 触发姿态控制
  // 绝对yaw角控制
  bool set_yaw_async(double target_yaw);
  bool set_yaw_block(double target_yaw);
  // 相对当前时刻yaw角控制
  bool set_yaw_adjust_async(double adjust_yaw);
  bool set_yaw_adjust_block(double adjust_yaw);
  // -------------------------高级控制接口--------------------------------
	// 高级控制接口全部使用异步接口，即时生效
  /**
   * @brief 在高级控制接口的速度控制中，我们主要参考了RMTT 相关API设计
   * @see https://robomaster-dev.readthedocs.io/zh-cn/latest/
   * @note 控制接口主要为机体系下的速度控制和惯性系下的速度控制
   */
  
	/**
	 * @brief 带有保护区域的线速度控制:当无人机超出了protect_area时会切入悬停状态而非继续运动，可以看作是电子围栏的一种，主要目的是为了防止速度控制失误导致无人机代码控制陷入失控
	 * @param protect_area 以当前无人机位置为中心，xyz为中心到对应方向边的长度，构造一个空间中的矩形区域
	 * @note 当前仅保留接口说明。底层控制器/FSM 尚未接入该保护区域逻辑，调用会返回 false。
	 */ 
  bool set_velocity_area(Eigen::Vector3d protect_area);
	// 清除当前的速度控制模式保护区域参数。
	// 当前仅保留接口说明。底层控制器/FSM 尚未接入该保护区域逻辑，调用会返回 false。
  bool clear_velocity_area(void);
  // 触发线速度控制
  bool set_linear_velocity_async(Eigen::Vector3d target_velocity);
	// 当前仅保留接口说明。底层尚未提供完整角速度控制通路，调用会返回 false。
	bool set_anglar_velocity_async(Eigen::Vector3d target_velocity);
	
	// 姿态推力控制。
	// 当前仅保留接口说明。外部姿态/推力接管、预热和失效回切链路尚未实现，调用会返回 false。
	bool set_attitude_thrust_async(Eigen::Vector3d attitude,double turust);
	// 当前仅保留接口说明。外部 bodyrate/thrust 接管链路尚未实现，调用会返回 false。
	bool set_bodyrate_thrust_async(Eigen::Vector3d bodyrate,double thrust);
	// 由于姿态推力控制具有一定的风线性，因此要求先进行预热,预热指的是，使用上面两个函数输出，状态机接受到的msg频率达到100Hz，并且推力数据与当前悬停推力一致，姿态数据或者bodyrate与当前悬停状态一致，持续3s
	// 当持续3s过后，状态机将会切断控制器的输出，转而使用helper传入的控制量，下面的函数返回true，在未预热或者预热过程中，返回false
	// 但值得注意的是，当外部控制的频率低于80Hz或者0.1s内没有输入，都会切回状态机内部的控制器，并且会在本次飞行过程中拒绝切换到外部控制
	// 当前仅保留接口说明。由于上述机制尚未落地实现，因此当前恒返回 false。
	bool is_external_attitude_thrust_ready();

  // 触发轨迹控制
  // TODO: 添加轨迹 数据类型
  bool set_trajectory_point_async(const uav_control::TrajectoryPoint &target_trajpoint);
	bool set_trajectory_async(const uav_control::Trajectory &trajectory);

	// 复合控制模式，使用mavros系列掩码，使用轨迹点数据类型，example: 允许用户同时控制xy位置和z轴速度
  // 当前仅保留接口说明。FSM 已接收 envelope，但尚未连到实际控制输出通路，调用会返回 false。
  bool set_complex_control();

  // -------------------------查询接口--------------------------------
  // 无人机当前状态
  uav_control::UAVStateEstimate get_uav_odometry();
  Eigen::Vector3d get_uav_position();
  Eigen::Vector3d get_uav_velocity_linear();
  Eigen::Vector3d get_uav_velocity_angular();
  Eigen::Vector3d get_uav_attitude_rpy_rad();
  Eigen::Vector3d get_uav_attitude_rpy_deg();
  double get_uav_yaw_rad();
  double get_uav_yaw_deg();
  Eigen::Quaterniond get_uav_attitude_quat();
  // 得到设定的目标状态
  Eigen::Vector3d get_target_position();
  Eigen::Vector3d get_target_velocity_linear();
  Eigen::Vector3d get_target_velocity_angular();
  Eigen::Vector3d get_target_attitude_rpy_rad();
  Eigen::Vector3d get_target_attitude_rpy_deg();
  double get_target_yaw_rad();
  double get_target_yaw_deg();
  Eigen::Quaterniond get_target_attitude_quat();
  // 当前仅对外部 thrust 控制链路有明确语义；由于该链路尚未实现，目前默认返回 0.0。
  double get_target_thrust();
  // Sunray FSM状态
  // TODO:实现Sunray状态机 状态的数据类型,本质上是强类型枚举
  sunray_fsm::SunrayState get_statemachine_state();
  //

private:
  // 手动更新本地缓存的 FSM 状态
  void set_cached_fsm_state(sunray_fsm::SunrayState state);
  // FSM 状态回调
  void fsm_state_cb(const std_msgs::UInt8::ConstPtr &msg);
  // 等待FSM的状态
  bool wait_for_fsm_state(sunray_fsm::SunrayState expected_state,
                          double timeout_s);
  // 等待达到期望的位置
  bool wait_for_position_reached(const Eigen::Vector3d &target_position,
                                 double timeout_s);
  // 等待位置控制命令完成
  bool wait_for_position_control_completed(double timeout_s);
  // 等待yaw到达期望值
  bool wait_for_yaw_reached(double target_yaw, double timeout_s);
  // 等待降落结束
  bool wait_for_landed(double timeout_s);
  // 在 FSM 已完成后，尽力确认 PX4 landed/disarmed，但不再作为硬失败条件
  void confirm_landed_best_effort(double timeout_s, const char *context);
  // 等待返航流程完整结束（包含返航后的降落）
  bool wait_for_return_completed(double timeout_s);

  ros::NodeHandle nh_;
  ros::NodeHandle ctrl_nh_;
  std::string uav_ns_;

  std::unique_ptr<PX4_DataReader> px4_data_reader_;
  bool px4_reader_ready_{false};
  double takeoff_wait_timeout_s_{12.0};
  double position_reached_tolerance_m_{0.1};
  double block_wait_timeout_s_{10.0};
  double land_wait_timeout_s_{20.0};
  double land_confirm_timeout_s_{3.0};
  double wait_poll_hz_{20.0};
  double yaw_reached_tolerance_rad_{0.0872664626};
  double trajectory_nominal_speed_mps_{0.5};

  // 里程计消息缓存
  uav_control::UAVStateEstimate uav_odometry_;
  //  无人机目标状态缓存
  uav_control::UAVStateEstimate uav_target_;
  // 状态机状态缓存
  mutable std::mutex fsm_state_mutex_;
  sunray_fsm::SunrayState fsm_state_{sunray_fsm::SunrayState::OFF};
  bool fsm_state_received_{false};

  // 声明与Sunray_FSM相关的发布者
  // 触发模式相关
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher return_pub_;
  // 控制接口相关
  ros::Publisher position_cmd_pub_;
  ros::Publisher velocity_cmd_pub_;
  ros::Publisher attitude_cmd_pub_;
  ros::Publisher trajectory_envelope_pub_;
  ros::Publisher complex_cmd_pub_;
  ros::Subscriber fsm_state_sub_;
  // 服务客户端(通过服务实现的控制，都是阻塞的方式)
  ros::ServiceClient takeoff_client_;
  ros::ServiceClient land_client_;
  ros::ServiceClient return_client_;
  ros::ServiceClient position_cmd_client_;
};
