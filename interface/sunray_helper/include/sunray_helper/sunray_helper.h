/**
 * @file sunray_helper.h
 * @brief
 *1.由于辅助类本身就是在帮住用户封装复杂的流程，因此原Sunray项目中的auto_takeoff命名显得有点冗杂，
        于是我们去掉了auto,但auto_return就变成了return，而return是编程的关键词，不能作为函数名，因此我们修改为return_home
 *2.在原来的接口上我们添加了_wait_for_complete构造新的函数，新函数旨在表明等待该过程直到结束，
 *3.由于这里的函数名有的会有一些冗长，但是自动format工具会导致可读性略有降低，因此本文件选择手动对齐
 ** @author Taolinyinjiu@YunDrone Tech (sirui@yundrone.com)
 * @date 2026-03-17
 * @version 0.1
 *
 */
#pragma once

#include "helper_config.hpp"
#include "position_point.hpp"
#include "sunray_common/quad_state_estimate.h"
#include <cstdint>
#include <optional>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>
#include <vector>

enum class SunrayTaskStatus : uint8_t {
  Idle = 0,
  Running = 1,
  Succeeded = 2,
  Failed = 3
};

struct SunrayTaskInfo {
  std::string task_name;
  SunrayTaskStatus status = SunrayTaskStatus::Idle;
  double progress = 0.0;
  bool completed = false;
};

struct SunrayUavInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ros::Time stamp;
  sunray_common::QuadStateEstimate odom;
  std::string state_machine_state;
  std::string latest_log;
  std::vector<SunrayTaskInfo> tasks;

  bool isValid() const { return odom.isValid(); }

  bool hasRunningTask() const {
    for (const auto &task : tasks) {
      if (task.status == SunrayTaskStatus::Running) {
        return true;
      }
    }
    return false;
  }

  std_msgs::String toRosMessage() const {
    auto task_status_to_string = [](SunrayTaskStatus status) -> const char * {
      switch (status) {
      case SunrayTaskStatus::Idle:
        return "idle";
      case SunrayTaskStatus::Running:
        return "running";
      case SunrayTaskStatus::Succeeded:
        return "succeeded";
      case SunrayTaskStatus::Failed:
        return "failed";
      }
      return "unknown";
    };

    std_msgs::String msg;
    std::ostringstream oss;
    oss << "stamp=" << stamp.toSec()
        << ";position=[" << odom.position.x() << "," << odom.position.y() << ","
        << odom.position.z() << "]"
        << ";velocity=[" << odom.velocity.x() << "," << odom.velocity.y() << ","
        << odom.velocity.z() << "]"
        << ";orientation=[" << odom.orientation.w() << "," << odom.orientation.x()
        << "," << odom.orientation.y() << "," << odom.orientation.z() << "]"
        << ";bodyrates=[" << odom.bodyrates.x() << "," << odom.bodyrates.y()
        << "," << odom.bodyrates.z() << "]"
        << ";state_machine_state=" << state_machine_state
        << ";latest_log=" << latest_log
        << ";task_count=" << tasks.size();

    for (size_t index = 0; index < tasks.size(); ++index) {
      const auto &task = tasks[index];
      oss << ";task[" << index << "].name=" << task.task_name
          << ";task[" << index << "].status="
          << task_status_to_string(task.status)
          << ";task[" << index << "].progress=" << task.progress
          << ";task[" << index << "].completed="
          << (task.completed ? "true" : "false");
    }

    msg.data = oss.str();
    return msg;
  }
};

class SunrayHelper {
public:
  // 构造函数
  explicit SunrayHelper(ros::NodeHandle &nh);
  // 析构函数
  ~SunrayHelper();

  SunrayUavInfo get_uav_info();
  // -------------------------起飞接口--------------------------------
  bool takeoff(std::optional<double> relative_takeoff_height = std::nullopt,
               std::optional<double> max_takeoff_velocity = std::nullopt);
  bool takeoff_wait_for_complete(
      std::optional<double> relative_takeoff_height = std::nullopt,
      std::optional<double> max_takeoff_velocity = std::nullopt);

  // -------------------------降落接口--------------------------------
  bool land(std::optional<int> land_type = std::nullopt,
            std::optional<double> land_max_velocity = std::nullopt);
  bool land_wait_for_complete(
      std::optional<int> land_type = std::nullopt,
      std::optional<double> land_max_velocity = std::nullopt);

  // -------------------------返航接口--------------------------------
  // 由于return本质上是位置控制+降落，因此我们支持位置控制的调节参数+降落的调节参数
  bool
  return_home(std::optional<Eigen::Vector3d> target_position = std::nullopt,
              std::optional<double> target_yaw = std::nullopt,
              std::optional<int> land_type = std::nullopt,
              std::optional<double> land_max_velocity = std::nullopt);
  bool return_home_wait_for_complete(
      std::optional<Eigen::Vector3d> target_position = std::nullopt,
      std::optional<double> target_yaw = std::nullopt,
      std::optional<int> land_type = std::nullopt,
      std::optional<double> land_max_velocity = std::nullopt);
  // -------------------------基础移动接口--------------------------------
  // 在移动的过程中，位置是一定要填写的，yaw角是可以不填的
  bool move_to(const MovePoint &point);
  bool move_to(Eigen::Vector3d position,
               std::optional<double> velocity = std::nullopt,
               std::optional<double> yaw = std::nullopt,
               std::optional<double> yaw_rate = std::nullopt);
  bool move_to_wait_for_complete(const MovePoint &point);
  bool move_to_wait_for_complete(Eigen::Vector3d position,
                                 std::optional<double> velocity = std::nullopt,
                                 std::optional<double> yaw = std::nullopt,
                                 std::optional<double> yaw_rate = std::nullopt);
  // 修改当前yaw角，默认使用绝对yaw角，当relative = true时，设置为相对yaw角
  bool set_yaw(double yaw, bool relative = false);
  bool set_yaw_wait_for_complete(double yaw, bool relative = false);

private:
  // 保存句柄
  ros::NodeHandle nh_;
  // 构造订阅者
};
// namespace sunray_helper
