#include "sunray_statemachine/sunray_statemachine.h"

#include <ros/ros.h>

namespace {

bool wait_for_fsm_namespace_params(double timeout_s) {
  const ros::WallTime start_time = ros::WallTime::now();
  const ros::WallDuration timeout(std::max(0.0, timeout_s));
  bool reported_waiting_global = false;
  bool reported_waiting_namespaced = false;

  while (ros::ok() && (ros::WallTime::now() - start_time) <= timeout) {
    std::string uav_name;
    int uav_id = 0;
    if (ros::param::get("/uav_name", uav_name) && !uav_name.empty() &&
        ros::param::get("/uav_id", uav_id)) {
      const std::string uav_ns = "/" + uav_name + std::to_string(uav_id);

      std::string namespaced_uav_name;
      int namespaced_uav_id = 0;
      if (ros::param::get(uav_ns + "/uav_name", namespaced_uav_name) &&
          !namespaced_uav_name.empty() &&
          ros::param::get(uav_ns + "/uav_id", namespaced_uav_id)) {
        ROS_INFO("[sunray_fsm_node] parameter namespace is ready: %s",
                 uav_ns.c_str());
        return true;
      }

      if (!reported_waiting_namespaced) {
        ROS_INFO("[sunray_fsm_node] waiting for namespaced params under %s",
                 uav_ns.c_str());
        reported_waiting_namespaced = true;
      }
    } else if (!reported_waiting_global) {
      ROS_INFO("[sunray_fsm_node] waiting for /uav_name and /uav_id");
      reported_waiting_global = true;
    }

    ros::WallDuration(0.1).sleep();
  }

  return false;
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_fsm_node");
  ros::NodeHandle nh;

  double param_wait_timeout_s = 5.0;
  nh.param("/fsm/param_wait_timeout_s", param_wait_timeout_s,
           param_wait_timeout_s);
  if (!wait_for_fsm_namespace_params(param_wait_timeout_s)) {
    ROS_FATAL("[sunray_fsm_node] parameter wait timeout after %.2f s",
              param_wait_timeout_s);
    return 1;
  }

  sunray_fsm::Sunray_StateMachine fsm(nh);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate supervisor_rate(fsm.get_supervisor_update_hz());
  while (ros::ok()) {
    fsm.update_slow();
    supervisor_rate.sleep();
  }
  return 0;
}
