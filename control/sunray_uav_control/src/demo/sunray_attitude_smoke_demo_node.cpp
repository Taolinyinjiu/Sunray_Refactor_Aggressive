#include "sunray_helper/sunray_helper.h"

#include <ros/ros.h>

#include <Eigen/Dense>

namespace {

const char *fsm_state_to_string(sunray_fsm::SunrayState state) {
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
    return "UNKNOWN";
  }
}

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "sunray_attitude_smoke_demo_node");
  ros::NodeHandle nh;

  double takeoff_hover_s = 2.0;
  double move_hover_s = 1.0;
  double move_x_m = 1.0;
  double takeoff_relative_height_m = 0.6;
  nh.param("takeoff_hover_s", takeoff_hover_s, takeoff_hover_s);
  nh.param("move_hover_s", move_hover_s, move_hover_s);
  nh.param("move_x_m", move_x_m, move_x_m);
  nh.param("takeoff_relative_height", takeoff_relative_height_m,
           takeoff_relative_height_m);

  Sunray_Helper helper(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  const auto log_snapshot = [&helper](const char *stage,
                                      const char *ref_pos_label,
                                      const Eigen::Vector3d &ref_pos,
                                      const char *ref_rpy_label,
                                      const Eigen::Vector3d &ref_rpy_deg) {
    const Eigen::Vector3d pos = helper.get_uav_position();
    const Eigen::Vector3d vel = helper.get_uav_velocity_linear();
    const Eigen::Vector3d rpy_deg = helper.get_uav_attitude_rpy_deg();
    const sunray_fsm::SunrayState fsm_state = helper.get_statemachine_state();
    ROS_INFO(
        "[SunrayAttitudeSmokeDemo] %s | fsm=%s | pos=(%.3f, %.3f, %.3f) "
        "vel=(%.3f, %.3f, %.3f) rpy_deg=(%.1f, %.1f, %.1f) "
        "%s=(%.3f, %.3f, %.3f) %s=(%.1f, %.1f, %.1f)",
        stage, fsm_state_to_string(fsm_state), pos.x(), pos.y(), pos.z(),
        vel.x(), vel.y(), vel.z(), rpy_deg.x(), rpy_deg.y(), rpy_deg.z(),
        ref_pos_label, ref_pos.x(), ref_pos.y(), ref_pos.z(), ref_rpy_label,
        ref_rpy_deg.x(), ref_rpy_deg.y(), ref_rpy_deg.z());
  };

  const auto fail_with_snapshot = [&log_snapshot](const char *stage,
                                                  const char *ref_pos_label,
                                                  const Eigen::Vector3d &ref_pos,
                                                  const char *ref_rpy_label,
                                                  const Eigen::Vector3d &ref_rpy_deg) {
    ROS_WARN("[SunrayAttitudeSmokeDemo] %s failed", stage);
    log_snapshot(stage, ref_pos_label, ref_pos, ref_rpy_label, ref_rpy_deg);
  };

  const Eigen::Vector3d takeoff_start = helper.get_uav_position();
  const Eigen::Vector3d takeoff_target =
      takeoff_start + Eigen::Vector3d(0.0, 0.0, takeoff_relative_height_m);
  const Eigen::Vector3d takeoff_target_rpy_deg = helper.get_uav_attitude_rpy_deg();
  ROS_INFO("[SunrayAttitudeSmokeDemo] takeoff");
  log_snapshot("before_takeoff", "takeoff_target_pos", takeoff_target,
               "takeoff_target_rpy_deg", takeoff_target_rpy_deg);
  if (!helper.takeoff_block()) {
    fail_with_snapshot("takeoff", "takeoff_target_pos", takeoff_target,
                       "takeoff_target_rpy_deg", takeoff_target_rpy_deg);
    ros::shutdown();
    return 1;
  }
  log_snapshot("after_takeoff", "takeoff_target_pos", takeoff_target,
               "takeoff_target_rpy_deg", takeoff_target_rpy_deg);

  ROS_INFO("[SunrayAttitudeSmokeDemo] hover %.1fs", takeoff_hover_s);
  ros::Duration(takeoff_hover_s).sleep();
  log_snapshot("after_takeoff_hover", "takeoff_target_pos", takeoff_target,
               "takeoff_target_rpy_deg", takeoff_target_rpy_deg);

  const Eigen::Vector3d start = helper.get_uav_position();
  const Eigen::Vector3d target = start + Eigen::Vector3d(move_x_m, 0.0, 0.0);
  const Eigen::Vector3d move_target_rpy_deg = helper.get_target_attitude_rpy_deg();
  ROS_INFO("[SunrayAttitudeSmokeDemo] move_to (%.2f, %.2f, %.2f)", target.x(),
           target.y(), target.z());
  log_snapshot("before_move_to", "move_target_pos", target, "move_target_rpy_deg",
               move_target_rpy_deg);
  if (!helper.set_position_block(target)) {
    fail_with_snapshot("move_to", "move_target_pos", target,
                       "move_target_rpy_deg", move_target_rpy_deg);
    ros::shutdown();
    return 1;
  }
  log_snapshot("after_move_to", "move_target_pos", target, "move_target_rpy_deg",
               move_target_rpy_deg);

  ROS_INFO("[SunrayAttitudeSmokeDemo] hover %.1fs", move_hover_s);
  ros::Duration(move_hover_s).sleep();
  log_snapshot("after_move_hover", "move_target_pos", target,
               "move_target_rpy_deg", move_target_rpy_deg);

  const Eigen::Vector3d land_lock_pos = helper.get_uav_position();
  const Eigen::Vector3d land_lock_rpy_deg = helper.get_uav_attitude_rpy_deg();
  ROS_INFO("[SunrayAttitudeSmokeDemo] land");
  log_snapshot("before_land", "land_lock_pos", land_lock_pos,
               "land_lock_rpy_deg", land_lock_rpy_deg);
  if (!helper.land_block()) {
    fail_with_snapshot("land", "land_lock_pos", land_lock_pos,
                       "land_lock_rpy_deg", land_lock_rpy_deg);
    ros::shutdown();
    return 1;
  }
  log_snapshot("after_land", "land_lock_pos", land_lock_pos,
               "land_lock_rpy_deg", land_lock_rpy_deg);
  const Eigen::Vector3d landed_pos = helper.get_uav_position();
  const Eigen::Vector3d landing_delta = landed_pos - land_lock_pos;
  ROS_INFO("[SunrayAttitudeSmokeDemo] landing_delta_from_lock | dpos=(%.3f, %.3f, %.3f)",
           landing_delta.x(), landing_delta.y(), landing_delta.z());

  ROS_INFO("[SunrayAttitudeSmokeDemo] done");
  ros::shutdown();
  return 0;
}
