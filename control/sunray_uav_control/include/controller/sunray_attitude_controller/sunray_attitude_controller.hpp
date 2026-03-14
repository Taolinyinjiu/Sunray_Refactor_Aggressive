#pragma once

#include "controller/base_controller/base_controller.hpp"

namespace uav_control {

class Attitude_Controller : public Base_Controller {
public:
  bool load_param(ros::NodeHandle &nh);
  ControllerOutput update(void) override;

private:
  struct DesiredState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    double yaw{0.0};
  };

  struct ControlParam {
    double mass_kg{0.96};
    double gravity_mps2{9.81};
    double hover_percent{0.37};
    double tilt_angle_max_rad{20.0 * 3.14159265358979323846 / 180.0};
    Eigen::Vector3d int_max{Eigen::Vector3d(10.0, 10.0, 10.0)};
    Eigen::Vector3d Kp{Eigen::Vector3d(3.0, 3.0, 3.0)};
    Eigen::Vector3d Kv{Eigen::Vector3d(3.0, 3.0, 3.0)};
    Eigen::Vector3d Kvi{Eigen::Vector3d(0.3, 0.3, 0.3)};
    double controller_update_hz{100.0};
    double max_position_error_m{3.0};
    double max_velocity_error_mps{3.0};
    double position_integral_start_error_xy_m{0.2};
    double position_integral_start_error_z_m{1.0};
    double min_command_thrust{0.1};
  };

  void reset_takeoff_context_if_needed();
  void reset_land_context_if_needed();
  void reset_integrator();
  DesiredState build_desired_state(
      const TrajectoryPointReference &trajectory_ref) const;
  ControllerOutput solve_attitude_thrust(
      const TrajectoryPointReference &trajectory_ref,
      bool allow_xy_integral);
  ControllerOutput solve_attitude_thrust(const DesiredState &desired_state,
                                         bool allow_xy_integral);

  ControllerOutput handle_undefined_state();
  ControllerOutput handle_off_state();
  ControllerOutput handle_takeoff_state();
  ControllerOutput handle_hover_state();
  ControllerOutput handle_move_state();
  ControllerOutput handle_land_state();
  ControlParam ctrl_param_{};
  Eigen::Vector3d int_e_v_{Eigen::Vector3d::Zero()};
  ros::Time last_update_time_{};
  double takeoff_yaw_{0.0};
};

} // namespace uav_control
