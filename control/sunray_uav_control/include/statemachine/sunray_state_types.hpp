/**
 * @file sunray_state_types.hpp
 * @brief 
 * @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 * 
 */

#pragma once

namespace sunray_fsm{
	// sunray_fsm的状态集合
	enum class SunrayState {
  OFF = 0,            ///< 待机/未激活状态。
  TAKEOFF,            ///< 起飞过程状态。
  HOVER,              ///< 悬停状态（主稳态）。
  RETURN,             ///< 返航状态
  LAND,               ///< 降落过程状态。
  EMERGENCY_LAND,     ///< 紧急降落状态。
  POSITION_CONTROL,   ///< 位置控制模式
  VELOCITY_CONTROL,   ///< 速度控制模式。
  ATTITUDE_CONTROL,   ///< 姿态控制模式。
  COMPLEX_CONTROL,    ///< 复合控制模式。
  TRAJECTORY_CONTROL, ///< 轨迹控制模式
};

// 用于触发状态转移的事件集合
enum class SunrayEvent {
  TAKEOFF_REQUEST = 0,      ///< 请求起飞。
  TAKEOFF_COMPLETED,        ///< 起飞完成。
  LAND_REQUEST,             ///< 请求降落。
  LAND_COMPLETED,           ///< 降落完成。
  EMERGENCY_REQUEST,        ///< 请求紧急降落。
  EMERGENCY_COMPLETED,      ///< 紧急降落完成。
  RETURN_REQUEST,           ///< 请求返航。
  RETURN_COMPLETED,         ///< 返航完成。
  WATCHDOG_ERROR,           ///< 看门狗异常。
  ENTER_POSITION_CONTROL,   ///< 进入位置控制
  ENTER_VELOCITY_CONTROL,   ///< 进入速度控制。
  ENTER_ATTITUDE_CONTROL,   ///< 进入姿态控制
  ENTER_COMPLEX_CONTROL,    ///< 进入复合控制
  ENTER_TRAJECTORY_CONTROL, ///< 进入轨迹控制
  POSITION_COMPLETED,       ///< 位置控制完成
  VELOCITY_COMPLETED,       ///< 速度控制完成
  ATTITUDE_COMPLETED,       ///< 姿态控制完成
  COMPLEX_COMPLETED,        ///< 复合控制完成
  TRAJECTORY_COMPLETED,     ///< 轨迹执行完成。
};


}