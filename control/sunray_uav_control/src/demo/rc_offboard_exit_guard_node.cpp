#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <string>

namespace {

std::string normalize_ns(const std::string &ns) {
  if (!ns.empty() && ns.front() == '/') {
    return ns.substr(1);
  }
  return ns;
}

std::string resolve_uav_namespace(ros::NodeHandle &nh) {
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

class RcOffboardExitGuard {
public:
  RcOffboardExitGuard(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh) {
    std::string private_uav_ns;
    if (pnh_.getParam("uav_ns", private_uav_ns) && !private_uav_ns.empty()) {
      uav_ns_ = normalize_ns(private_uav_ns);
    } else {
      uav_ns_ = resolve_uav_namespace(nh_);
    }
    pnh_.param("rc_channel", rc_channel_, rc_channel_);
    pnh_.param("rc_low_threshold", rc_low_threshold_, rc_low_threshold_);
    pnh_.param("rc_high_threshold", rc_high_threshold_, rc_high_threshold_);
    pnh_.param("target_mode", target_mode_, target_mode_);
    pnh_.param("require_offboard", require_offboard_, require_offboard_);
    pnh_.param("request_cooldown_s", request_cooldown_s_, request_cooldown_s_);

    if (rc_channel_ < 1) {
      rc_channel_ = 7;
    }
    if (rc_high_threshold_ <= rc_low_threshold_) {
      rc_low_threshold_ = 1300;
      rc_high_threshold_ = 1700;
    }
    if (request_cooldown_s_ < 0.0) {
      request_cooldown_s_ = 0.0;
    }

    const std::string mavros_ns =
        uav_ns_.empty() ? "/mavros" : ("/" + uav_ns_ + "/mavros");
    rc_sub_ = nh_.subscribe(mavros_ns + "/rc/in", 10,
                            &RcOffboardExitGuard::rc_cb, this);
    state_sub_ = nh_.subscribe(mavros_ns + "/state", 10,
                               &RcOffboardExitGuard::state_cb, this);
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
    monitor_timer_ =
        nh_.createTimer(ros::Duration(1.0),
                        &RcOffboardExitGuard::monitor_timer_cb, this);

    ROS_INFO(
        "[RcOffboardExitGuard] started, uav_ns='%s' rc_channel=%d "
        "thresholds=[%d,%d] target_mode=%s require_offboard=%d",
        uav_ns_.c_str(), rc_channel_, rc_low_threshold_, rc_high_threshold_,
        target_mode_.c_str(), static_cast<int>(require_offboard_));
  }

private:
  enum class SwitchState { UNKNOWN, LOW, MID, HIGH };

  SwitchState decode_switch_state(uint16_t pwm) const {
    if (pwm == 0U) {
      return SwitchState::UNKNOWN;
    }
    if (static_cast<int>(pwm) <= rc_low_threshold_) {
      return SwitchState::LOW;
    }
    if (static_cast<int>(pwm) >= rc_high_threshold_) {
      return SwitchState::HIGH;
    }
    return SwitchState::MID;
  }

  const char *switch_state_to_string(SwitchState state) const {
    switch (state) {
    case SwitchState::LOW:
      return "LOW";
    case SwitchState::MID:
      return "MID";
    case SwitchState::HIGH:
      return "HIGH";
    default:
      return "UNKNOWN";
    }
  }

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    if (!msg) {
      return;
    }
    current_mode_ = msg->mode;
    state_received_ = true;
  }

  void rc_cb(const mavros_msgs::RCIn::ConstPtr &msg) {
    if (!msg) {
      return;
    }

    last_rc_stamp_ = ros::Time::now();
    const std::size_t channel_index =
        static_cast<std::size_t>(std::max(1, rc_channel_) - 1);
    if (channel_index >= msg->channels.size()) {
      ROS_WARN_THROTTLE(
          1.0,
          "[RcOffboardExitGuard] rc channel %d is out of range, rc size=%zu",
          rc_channel_, msg->channels.size());
      return;
    }

    const uint16_t pwm = msg->channels[channel_index];
    const SwitchState current_state = decode_switch_state(pwm);

    if (current_state != last_switch_state_) {
      ROS_INFO("[RcOffboardExitGuard] channel %d pwm=%u switch=%s -> %s",
               rc_channel_, pwm, switch_state_to_string(last_switch_state_),
               switch_state_to_string(current_state));
    }

    const bool rising_edge_to_high =
        (last_switch_state_ == SwitchState::LOW &&
         current_state == SwitchState::HIGH);
    const bool hold_high_override =
        (current_state == SwitchState::HIGH && state_received_ &&
         current_mode_ == "OFFBOARD");

    if (rising_edge_to_high || hold_high_override) {
      request_target_mode(pwm);
    }

    last_switch_state_ = current_state;
  }

  void request_target_mode(uint16_t pwm) {
    const ros::Time now = ros::Time::now();
    if (!last_mode_request_time_.isZero() &&
        (now - last_mode_request_time_).toSec() < request_cooldown_s_) {
      ROS_WARN_THROTTLE(
          1.0,
          "[RcOffboardExitGuard] mode request skipped by cooldown, current_mode=%s",
          current_mode_.c_str());
      return;
    }

    if (require_offboard_ && state_received_ && current_mode_ != "OFFBOARD") {
      ROS_WARN_THROTTLE(
          1.0,
          "[RcOffboardExitGuard] trigger ignored because current mode is '%s' "
          "instead of OFFBOARD",
          current_mode_.c_str());
      return;
    }

    if (!set_mode_client_.exists() &&
        !set_mode_client_.waitForExistence(ros::Duration(0.5))) {
      ROS_WARN_THROTTLE(
          1.0,
          "[RcOffboardExitGuard] mavros set_mode service unavailable");
      return;
    }

    mavros_msgs::SetMode srv;
    srv.request.custom_mode = target_mode_;
    if (set_mode_client_.call(srv) && srv.response.mode_sent) {
      ROS_WARN(
          "[RcOffboardExitGuard] RC trigger accepted, channel %d pwm=%u "
          "request mode=%s current_mode=%s",
          rc_channel_, pwm, target_mode_.c_str(), current_mode_.c_str());
    } else {
      ROS_WARN("[RcOffboardExitGuard] RC trigger failed, channel %d pwm=%u "
               "request mode=%s current_mode=%s",
               rc_channel_, pwm, target_mode_.c_str(), current_mode_.c_str());
    }
    last_mode_request_time_ = now;
  }

  void monitor_timer_cb(const ros::TimerEvent &) {
    if (last_rc_stamp_.isZero()) {
      ROS_WARN_THROTTLE(
          2.0,
          "[RcOffboardExitGuard] waiting for RC input on mavros rc topic");
      return;
    }

    const double rc_age_s = (ros::Time::now() - last_rc_stamp_).toSec();
    if (rc_age_s > 1.0) {
      ROS_WARN_THROTTLE(
          2.0,
          "[RcOffboardExitGuard] RC input is stale, last update %.2fs ago",
          rc_age_s);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber rc_sub_;
  ros::Subscriber state_sub_;
  ros::ServiceClient set_mode_client_;
  ros::Timer monitor_timer_;

  std::string uav_ns_;
  int rc_channel_{7};
  int rc_low_threshold_{1300};
  int rc_high_threshold_{1700};
  std::string target_mode_{"STABILIZED"};
  bool require_offboard_{true};
  double request_cooldown_s_{0.2};

  bool state_received_{false};
  std::string current_mode_;
  ros::Time last_rc_stamp_;
  ros::Time last_mode_request_time_;
  SwitchState last_switch_state_{SwitchState::UNKNOWN};
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "rc_offboard_exit_guard_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RcOffboardExitGuard node(nh, pnh);
  ros::spin();
  return 0;
}
