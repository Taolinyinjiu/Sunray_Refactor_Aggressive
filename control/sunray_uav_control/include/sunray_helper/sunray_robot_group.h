#pragma once

#include "sunray_helper/sunray_helper.h"

#include <uav_control/Land.h>
#include <uav_control/PositionRequest.h>
#include <uav_control/ReturnHome.h>
#include <uav_control/Takeoff.h>
#include <uav_control/Trajectory.h>

#include <Eigen/Dense>
#include <ros/ros.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace sunray_helper_fluent {
namespace {

inline std::string normalize_ns(const std::string &ns) {
  if (!ns.empty() && ns.front() == '/') {
    return ns.substr(1);
  }
  return ns;
}

inline std::string resolve_uav_namespace(ros::NodeHandle &nh) {
  std::string key;
  std::string ns;
  if (nh.searchParam("uav_ns", key) && nh.getParam(key, ns) && !ns.empty()) {
    return normalize_ns(ns);
  }

  std::string name;
  int id = 0;
  bool ok_name = false;
  bool ok_id = false;
  if (nh.searchParam("uav_name", key)) {
    ok_name = nh.getParam(key, name) && !name.empty();
  }
  if (nh.searchParam("uav_id", key)) {
    ok_id = nh.getParam(key, id);
  }
  if (ok_name && ok_id) {
    return name + std::to_string(id);
  }

  name = "uav";
  id = 1;
  nh.param("/uav_name", name, name);
  nh.param("/uav_id", id, id);
  return name + std::to_string(id);
}

inline const char *to_string(sunray_fsm::SunrayState state) {
  switch (state) {
  case sunray_fsm::SunrayState::OFF:
    return "OFF";
  case sunray_fsm::SunrayState::TAKEOFF:
    return "TAKEOFF";
  case sunray_fsm::SunrayState::HOVER:
    return "HOVER";
  case sunray_fsm::SunrayState::RETURN:
    return "RETURN";
  case sunray_fsm::SunrayState::LAND:
    return "LAND";
  case sunray_fsm::SunrayState::EMERGENCY_LAND:
    return "EMERGENCY_LAND";
  case sunray_fsm::SunrayState::POSITION_CONTROL:
    return "POSITION_CONTROL";
  case sunray_fsm::SunrayState::VELOCITY_CONTROL:
    return "VELOCITY_CONTROL";
  case sunray_fsm::SunrayState::ATTITUDE_CONTROL:
    return "ATTITUDE_CONTROL";
  case sunray_fsm::SunrayState::COMPLEX_CONTROL:
    return "COMPLEX_CONTROL";
  case sunray_fsm::SunrayState::TRAJECTORY_CONTROL:
    return "TRAJECTORY_CONTROL";
  default:
    return "UNKNOWN_STATE";
  }
}

template <typename ServiceT>
inline bool dispatch_request(ros::ServiceClient *client, ServiceT *srv) {
  if (client == nullptr || srv == nullptr) {
    return false;
  }
  if (!client->exists() &&
      !client->waitForExistence(ros::Duration(1.0))) {
    return false;
  }
  if (!client->call(*srv)) {
    return false;
  }
  return srv->response.accepted;
}

} // namespace

class Sunray_TaskHandle {
public:
  Sunray_TaskHandle() = default;

  Sunray_TaskHandle(std::string task_name, bool dispatched,
                    std::function<bool()> wait_fn)
      : state_(std::make_shared<State>(std::move(task_name), dispatched,
                                       std::move(wait_fn))) {}

  bool dispatched() const { return state_ && state_->dispatched; }

  bool wait_for_completed() const {
    if (!state_ || !state_->dispatched || !state_->wait_fn) {
      return false;
    }

    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->wait_called) {
      state_->wait_result = state_->wait_fn();
      state_->wait_called = true;
    }
    return state_->wait_result;
  }

  const std::string &task_name() const {
    static const std::string kEmptyName;
    return state_ ? state_->task_name : kEmptyName;
  }

private:
  struct State {
    State(std::string task_name_in, bool dispatched_in,
          std::function<bool()> wait_fn_in)
        : task_name(std::move(task_name_in)), dispatched(dispatched_in),
          wait_fn(std::move(wait_fn_in)) {}

    std::string task_name;
    bool dispatched{false};
    std::function<bool()> wait_fn;
    mutable std::mutex mutex;
    bool wait_called{false};
    bool wait_result{false};
  };

  std::shared_ptr<State> state_;
};

class Sunray_RobotGroup {
public:
  explicit Sunray_RobotGroup(ros::NodeHandle &nh) {
    context_ = std::make_shared<Context>();
    context_->helper = std::make_shared<Sunray_Helper>(nh);

    const std::string uav_ns = resolve_uav_namespace(nh);
    const std::string ctrl_ns =
        uav_ns.empty() ? "/sunray_control" : ("/" + uav_ns + "/sunray_control");
    context_->ctrl_nh.reset(new ros::NodeHandle(ctrl_ns));
    context_->takeoff_client =
        context_->ctrl_nh->serviceClient<uav_control::Takeoff>("takeoff_request");
    context_->land_client =
        context_->ctrl_nh->serviceClient<uav_control::Land>("land_request");
    context_->return_client = context_->ctrl_nh->serviceClient<uav_control::ReturnHome>(
        "return_request");
    context_->position_client = context_->ctrl_nh->serviceClient<uav_control::PositionRequest>(
        "position_request");

    nh.param("takeoff_wait_s", context_->takeoff_wait_timeout_s,
             context_->takeoff_wait_timeout_s);
    nh.param("move_wait_s", context_->move_wait_timeout_s,
             context_->move_wait_timeout_s);
    nh.param("land_wait_s", context_->land_wait_timeout_s,
             context_->land_wait_timeout_s);
    nh.param("position_reached_tolerance_m", context_->position_tolerance_m,
             context_->position_tolerance_m);
    nh.param("wait_poll_hz", context_->wait_poll_hz, context_->wait_poll_hz);
    nh.param("return_wait_s", context_->return_wait_timeout_s,
             context_->return_wait_timeout_s);
  }

  Sunray_TaskHandle takeoff() { return takeoff(0.0, 0.0); }

  Sunray_TaskHandle takeoff(double relative_takeoff_height,
                            double max_takeoff_velocity) {
    uav_control::Takeoff srv;
    srv.request.takeoff_relative_height = relative_takeoff_height;
    srv.request.takeoff_max_velocity = max_takeoff_velocity;
    const bool dispatched = dispatch_request(&context_->takeoff_client, &srv);
    const std::shared_ptr<Context> context = context_;
    return Sunray_TaskHandle(
        "takeoff", dispatched,
        [context]() {
          return wait_for_state(context, sunray_fsm::SunrayState::HOVER,
                                context->takeoff_wait_timeout_s, "takeoff");
        });
  }

  Sunray_TaskHandle land() { return land(0, 0.0); }

  Sunray_TaskHandle land(int land_type, double land_max_velocity) {
    uav_control::Land srv;
    srv.request.land_type = land_type;
    srv.request.land_max_velocity = land_max_velocity;
    const bool dispatched = dispatch_request(&context_->land_client, &srv);
    const std::shared_ptr<Context> context = context_;
    return Sunray_TaskHandle(
        "land", dispatched,
        [context]() {
          return wait_for_state(context, sunray_fsm::SunrayState::OFF,
                                context->land_wait_timeout_s, "land");
        });
  }

  Sunray_TaskHandle move_to(const Eigen::Vector3d &position) {
    const bool dispatched = context_->helper->set_position_async(position);
    const std::shared_ptr<Context> context = context_;
    return Sunray_TaskHandle(
        "move_to", dispatched,
        [context]() {
          return wait_for_state(context, sunray_fsm::SunrayState::HOVER,
                                context->move_wait_timeout_s, "move_to");
        });
  }

  Sunray_TaskHandle move_to(const Eigen::Vector3d &position, double yaw) {
    const bool dispatched = context_->helper->set_position_async(position, yaw);
    const std::shared_ptr<Context> context = context_;
    return Sunray_TaskHandle(
        "move_to_with_yaw", dispatched,
        [context]() {
          return wait_for_state(context, sunray_fsm::SunrayState::HOVER,
                                context->move_wait_timeout_s,
                                "move_to_with_yaw");
        });
  }

  Sunray_TaskHandle return_home(double land_max_velocity = 0.0) {
    uav_control::ReturnHome srv;
    srv.request.use_takeoff_homepoint = true;
    srv.request.target_position.x = 0.0;
    srv.request.target_position.y = 0.0;
    srv.request.target_position.z = 0.0;
    srv.request.yaw_ctrl = false;
    srv.request.yaw = 0.0;
    srv.request.land_max_velocity = land_max_velocity;
    const bool dispatched = dispatch_request(&context_->return_client, &srv);
    const std::shared_ptr<Context> context = context_;
    return Sunray_TaskHandle(
        "return_home", dispatched,
        [context]() {
          return wait_for_state(context, sunray_fsm::SunrayState::OFF,
                                context->return_wait_timeout_s,
                                "return_home");
        });
  }

  Sunray_TaskHandle return_to(const Eigen::Vector3d &target_position,
                              bool yaw_ctrl = false, double yaw = 0.0,
                              double land_max_velocity = 0.0) {
    uav_control::ReturnHome srv;
    srv.request.use_takeoff_homepoint = false;
    srv.request.target_position.x = target_position.x();
    srv.request.target_position.y = target_position.y();
    srv.request.target_position.z = target_position.z();
    srv.request.yaw_ctrl = yaw_ctrl;
    srv.request.yaw = yaw;
    srv.request.land_max_velocity = land_max_velocity;
    const bool dispatched = dispatch_request(&context_->return_client, &srv);
    const std::shared_ptr<Context> context = context_;
    return Sunray_TaskHandle(
        "return_to", dispatched,
        [context]() {
          return wait_for_state(context, sunray_fsm::SunrayState::OFF,
                                context->return_wait_timeout_s, "return_to");
        });
  }

  Sunray_TaskHandle follow_trajectory(const uav_control::Trajectory &trajectory,
                                      double timeout_s) {
    const bool dispatched = context_->helper->set_trajectory_async(trajectory);
    const std::shared_ptr<Context> context = context_;
    const double effective_timeout =
        (timeout_s > 0.0) ? timeout_s : context->move_wait_timeout_s;
    return Sunray_TaskHandle(
        "follow_trajectory", dispatched,
        [context, effective_timeout]() {
          return wait_for_state(context, sunray_fsm::SunrayState::HOVER,
                                effective_timeout, "follow_trajectory");
        });
  }

  Sunray_Helper &raw_helper() { return *context_->helper; }
  const Sunray_Helper &raw_helper() const { return *context_->helper; }

private:
  struct Context {
    std::shared_ptr<Sunray_Helper> helper;
    std::shared_ptr<ros::NodeHandle> ctrl_nh;
    ros::ServiceClient takeoff_client;
    ros::ServiceClient land_client;
    ros::ServiceClient return_client;
    ros::ServiceClient position_client;
    double takeoff_wait_timeout_s{12.0};
    double move_wait_timeout_s{10.0};
    double land_wait_timeout_s{20.0};
    double return_wait_timeout_s{30.0};
    double position_tolerance_m{0.1};
    double wait_poll_hz{20.0};
  };

  static bool wait_for_state(const std::shared_ptr<Context> &context,
                             sunray_fsm::SunrayState expected_state,
                             double timeout_s, const char *task_name) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(std::max(0.0, timeout_s));
    ros::Rate rate(std::max(1.0, context->wait_poll_hz));

    while (ros::ok() && ros::WallTime::now() <= deadline) {
      ros::spinOnce();
      const sunray_fsm::SunrayState current_state =
          context->helper->get_statemachine_state();
      if (current_state == expected_state) {
        return true;
      }
      rate.sleep();
    }

    const sunray_fsm::SunrayState current_state =
        context->helper->get_statemachine_state();
    ROS_WARN("[SunrayRobotGroup] %s timeout: expected=%s current=%s timeout=%.2f",
             task_name, to_string(expected_state), to_string(current_state),
             timeout_s);
    return false;
  }

  static bool wait_for_position(const std::shared_ptr<Context> &context,
                                const Eigen::Vector3d &target_position,
                                double timeout_s, const char *task_name) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(std::max(0.0, timeout_s));
    ros::Rate rate(std::max(1.0, context->wait_poll_hz));

    while (ros::ok() && ros::WallTime::now() <= deadline) {
      ros::spinOnce();
      const Eigen::Vector3d current_position = context->helper->get_uav_position();
      if ((current_position - target_position).norm() <=
          context->position_tolerance_m) {
        return true;
      }
      rate.sleep();
    }

    const Eigen::Vector3d current_position = context->helper->get_uav_position();
    ROS_WARN("[SunrayRobotGroup] %s timeout: current=(%.3f, %.3f, %.3f) "
             "target=(%.3f, %.3f, %.3f) tol=%.3f timeout=%.2f",
             task_name, current_position.x(), current_position.y(),
             current_position.z(), target_position.x(), target_position.y(),
             target_position.z(), context->position_tolerance_m, timeout_s);
    return false;
  }

  std::shared_ptr<Context> context_;
};

using RobotGroup = Sunray_RobotGroup;
using TaskHandle = Sunray_TaskHandle;

} // namespace sunray_helper_fluent
