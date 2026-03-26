#include "mavros_helper/mavros_helper.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string BoolToString(bool value) {
    return value ? "true" : "false";
}

std::string FlightModeToString(control_common::FlightMode mode) {
    switch (mode) {
    case control_common::FlightMode::Manual:
        return "Manual";
    case control_common::FlightMode::Acro:
        return "Acro";
    case control_common::FlightMode::Altctl:
        return "Altctl";
    case control_common::FlightMode::Posctl:
        return "Posctl";
    case control_common::FlightMode::Offboard:
        return "Offboard";
    case control_common::FlightMode::Stabilized:
        return "Stabilized";
    case control_common::FlightMode::Rattitude:
        return "Rattitude";
    case control_common::FlightMode::AutoMission:
        return "AutoMission";
    case control_common::FlightMode::AutoLoiter:
        return "AutoLoiter";
    case control_common::FlightMode::AutoRtl:
        return "AutoRtl";
    case control_common::FlightMode::AutoLand:
        return "AutoLand";
    case control_common::FlightMode::AutoRtgs:
        return "AutoRtgs";
    case control_common::FlightMode::AutoReady:
        return "AutoReady";
    case control_common::FlightMode::AutoTakeoff:
        return "AutoTakeoff";
    case control_common::FlightMode::Undefined:
    default:
        return "Undefined";
    }
}

std::string LandedStateToString(control_common::LandedState state) {
    switch (state) {
    case control_common::LandedState::OnGround:
        return "OnGround";
    case control_common::LandedState::InAir:
        return "InAir";
    case control_common::LandedState::Takeoff:
        return "Takeoff";
    case control_common::LandedState::Landing:
        return "Landing";
    case control_common::LandedState::Undefined:
    default:
        return "Undefined";
    }
}

std::string LocalFrameToString(control_common::Mavros_SetpointLocal::Mavros_LocalFrame frame) {
    switch (frame) {
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Ned:
        return "Local_Ned";
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Local_Offset_Ned:
        return "Local_Offset_Ned";
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned:
        return "Body_Ned";
    case control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned:
        return "Body_Offset_Ned";
    default:
        return "Unknown";
    }
}

std::string VecToString(const Eigen::Vector3d& value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "[" << value.x() << ", " << value.y() << ", "
        << value.z() << "]";
    return oss.str();
}

std::string QuatToString(const Eigen::Quaterniond& value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << "[w=" << value.w() << ", x=" << value.x()
        << ", y=" << value.y() << ", z=" << value.z() << "]";
    return oss.str();
}

std::string TimeToString(const ros::Time& stamp) {
    if (stamp.isZero()) {
        return "0.000";
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << stamp.toSec();
    return oss.str();
}

std::string BuildReport(MavrosHelper& helper) {
    const bool ready = helper.is_ready();
    const control_common::Mavros_State state = helper.get_state();
    const control_common::Mavros_Battery battery = helper.get_battery();
    const control_common::Mavros_Estimator estimator = helper.get_estimator_status();
    const control_common::UAVStateEstimate odom = helper.get_odometry();
    const Eigen::Quaterniond attitude_quat = helper.get_attitude_quat();
    const Eigen::Vector3d attitude_euler_rad = helper.get_attitude_eluer_rad();
    const Eigen::Vector3d attitude_euler_deg = helper.get_attitude_eluer_deg();
    const control_common::Mavros_SetpointLocal target_local = helper.get_target_local();
    const control_common::Mavros_SetpointAttitude target_attitude = helper.get_target_attitude();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "\n========== MavrosHelper ==========\n";
    oss << "ready: " << BoolToString(ready) << "\n";
    oss << "[state] valid=" << BoolToString(state.valid)
        << " stamp=" << TimeToString(state.timestamp)
        << " connected=" << BoolToString(state.connected)
        << " armed=" << BoolToString(state.armed)
        << " rc_input=" << BoolToString(state.rc_input)
        << " system_status=" << static_cast<int>(state.system_status)
        << " mode=" << FlightModeToString(state.flight_mode)
        << " landed=" << LandedStateToString(state.landed_state)
        << " load=" << state.system_load
        << " voltage=" << state.voltage
        << " current=" << state.current
        << " percent=" << state.percent << "\n";
    oss << "[battery] valid=" << BoolToString(battery.valid)
        << " stamp=" << TimeToString(battery.timestamp)
        << " voltage=" << battery.voltage
        << " current=" << battery.current
        << " percent=" << battery.percent << "\n";
    oss << "[estimator] valid=" << BoolToString(estimator.valid)
        << " stamp=" << TimeToString(estimator.timestamp)
        << " attitude_valid=" << BoolToString(estimator.attitude_valid)
        << " local_horiz_valid=" << BoolToString(estimator.local_hroiz_valid)
        << " local_vertical_valid=" << BoolToString(estimator.local_vertical_valid)
        << " global_horiz_valid=" << BoolToString(estimator.global_hroiz_valid)
        << " global_vertical_valid=" << BoolToString(estimator.global_vertical_valid)
        << " gps_error=" << BoolToString(estimator.gps_error)
        << " acc_error=" << BoolToString(estimator.acc_error) << "\n";
    oss << "[odometry] stamp=" << TimeToString(odom.timestamp)
        << " position=" << VecToString(odom.position)
        << " orientation=" << QuatToString(odom.orientation)
        << " velocity=" << VecToString(odom.velocity)
        << " bodyrate=" << VecToString(odom.bodyrate) << "\n";
    oss << "[attitude] quat=" << QuatToString(attitude_quat)
        << " euler_rad=" << VecToString(attitude_euler_rad)
        << " euler_deg=" << VecToString(attitude_euler_deg)
        << " yaw_rad=" << helper.get_yaw_rad()
        << " yaw_deg=" << helper.get_yaw_deg() << "\n";
    oss << "[target_local] valid=" << BoolToString(target_local.valid)
        << " stamp=" << TimeToString(target_local.timestamp)
        << " frame=" << LocalFrameToString(target_local.frame)
        << " mask=" << target_local.mask
        << " position=" << VecToString(target_local.position)
        << " velocity=" << VecToString(target_local.velocity)
        << " accel_or_force=" << VecToString(target_local.accel_or_force)
        << " yaw=" << target_local.yaw
        << " yaw_rate=" << target_local.yaw_rate << "\n";
    oss << "[target_attitude] valid=" << BoolToString(target_attitude.valid)
        << " stamp=" << TimeToString(target_attitude.timestamp)
        << " mask=" << static_cast<int>(target_attitude.mask)
        << " attitude=" << QuatToString(target_attitude.attitude)
        << " body_rate=" << VecToString(target_attitude.body_rate)
        << " thrust=" << target_attitude.thrust << "\n";
    oss << "=================================\n";
    return oss.str();
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_helper_print_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    MavrosHelper_ConfigList config(false);
    config.state = pnh.param("state", true);
    config.battery = pnh.param("battery", true);
    config.ekf2_status = pnh.param("ekf2_status", true);
    config.local_odom = pnh.param("local_odom", true);
    config.uav_control_handle = pnh.param("uav_control_handle", false);
    config.uav_target_state = pnh.param("uav_target_state", true);

    const double print_hz = pnh.param("print_hz", 1.0);

    try {
        MavrosHelper helper(nh, config);
        ros::AsyncSpinner spinner(2);
        spinner.start();

        ROS_INFO_STREAM("mavros_helper_print_node started."
                        << " print_hz=" << print_hz
                        << " state=" << BoolToString(config.state)
                        << " battery=" << BoolToString(config.battery)
                        << " ekf2_status=" << BoolToString(config.ekf2_status)
                        << " local_odom=" << BoolToString(config.local_odom)
                        << " uav_target_state=" << BoolToString(config.uav_target_state)
                        << " uav_control_handle=" << BoolToString(config.uav_control_handle));

        ros::Rate rate(print_hz > 0.0 ? print_hz : 1.0);
        while (ros::ok()) {
            ROS_INFO_STREAM(BuildReport(helper));
            rate.sleep();
        }
    } catch (const std::runtime_error& e) {
        ROS_FATAL_STREAM("failed to construct MavrosHelper: " << e.what());
        return 1;
    }

    return 0;
}
