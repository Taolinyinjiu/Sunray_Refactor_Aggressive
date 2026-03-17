/**
 * @file sunray_config_types.hpp
 * @brief
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 *
 */

#pragma once

#include <string>

namespace sunray_fsm {
struct SunrayFSM_ParamConfig {
  // -------------------基本参数-----------------------
  std::string uav_name;          // 无人机名称
  int uav_id;                    // 无人机编号
  double mass_kg;                // 无人机质量
  double gravity;                // 重力加速度

  int controller_type;           // 控制器类型
  double controller_update_hz;   // 控制器更新频率(Hz)
  double supervisor_update_hz;   // 监督循环频率(Hz)

  std::string odom_topic_name;   // 里程计话题名
  bool fuse_odom_to_px4;         // 是否融合里程计到 PX4
  int fuse_odom_type;            // 里程计融合类型
  double fuse_odom_frequency_hz; // 里程计融合频率(Hz)

  double low_voltage_v;          // 低电压阈值(V)
  int low_voltage_action;        // 低电压动作

  bool control_with_no_rc;       // 无遥控器时是否允许控制
  int lost_with_rc_action;       // 遥控器失联动作

  bool arm_with_code;            // 是否允许代码解锁
  bool takeoff_with_code;        // 是否允许代码起飞
  bool check_flip;               // 是否检测倾倒

  double fence_x_max, fence_x_min; // 电子围栏 X 范围
  double fence_y_max, fence_y_min; // 电子围栏 Y 范围
  double fence_z_max, fence_z_min; // 电子围栏 Z 范围

  double timeout_odom_s;         // 里程计超时(s)
  double timeout_rc_s;           // 遥控器超时(s)
  double timeout_control_hb_s;   // 轨迹控制心跳超时(s)，超时切换为hover
  double timeout_imu_s;          // IMU 超时(s)
  double timeout_battery_s;      // 电池超时(s)

  double error_tolerance_pos_x_m; // X 位置误差容许(m)
  double error_tolerance_pos_y_m; // Y 位置误差容许(m)
  double error_tolerance_pos_z_m; // Z 位置误差容许(m)

  double max_velocity_x_mps;     // X 最大速度(m/s)
  double max_velocity_y_mps;     // Y 最大速度(m/s)
  double max_velocity_z_mps;     // Z 最大速度(m/s)

  double max_velocity_with_rc_x_mps; // RC 控制时 X 最大速度(m/s)
  double max_velocity_with_rc_y_mps; // RC 控制时 Y 最大速度(m/s)
  double max_velocity_with_rc_z_mps; // RC 控制时 Z 最大速度(m/s)

  double tilt_angle_max_deg;     // 最大倾角(deg)

  int land_type;                 // 降落模式

  double takeoff_height_m;       // 相对起飞高度(m)
  double takeoff_max_vel_mps;    // 起飞最大速度(m/s)
  double land_max_vel_mps;       // 降落最大速度(m/s)
};
} // namespace sunray_fsm
