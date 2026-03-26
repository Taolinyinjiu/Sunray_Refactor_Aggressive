#include <gtest/gtest.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SysStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>

#include "mavros_helper/mavros_helper.hpp"

namespace {

constexpr char kStateTopic[] = "/test_uav/1/mavros/state";
constexpr char kExtendedStateTopic[] = "/test_uav/1/mavros/extended_state";
constexpr char kSysStatusTopic[] = "/test_uav/1/mavros/sys_status";
constexpr char kBatteryTopic[] = "/test_uav/1/mavros/battery";
constexpr char kEstimatorTopic[] = "/test_uav/1/mavros/estimator_status";
constexpr char kOdometryTopic[] = "/test_uav/1/mavros/local_position/odom";
constexpr char kTargetLocalTopic[] = "/test_uav/1/mavros/setpoint_raw/target_local";
constexpr char kTargetAttitudeTopic[] = "/test_uav/1/mavros/setpoint_raw/target_attitude";
constexpr char kVisionPoseTopic[] = "/test_uav/1/mavros/vision_pose/pose";
constexpr char kLocalSetpointTopic[] = "/test_uav/1/mavros/setpoint_raw/local";
constexpr char kAttitudeSetpointTopic[] = "/test_uav/1/mavros/setpoint_raw/attitude";
constexpr char kArmService[] = "/test_uav/1/mavros/cmd/arming";
constexpr char kModeService[] = "/test_uav/1/mavros/set_mode";

template <typename Predicate>
bool WaitForCondition(const Predicate& predicate, double timeout_sec = 2.0) {
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
    while (ros::ok() && ros::Time::now() < deadline) {
        if (predicate()) {
            return true;
        }
        ros::Duration(0.01).sleep();
    }
    return predicate();
}

bool WaitForSubscriberConnection(const ros::Publisher& publisher) {
    return WaitForCondition([&publisher]() { return publisher.getNumSubscribers() >= 1; });
}

bool WaitForPublisherConnection(const ros::Subscriber& subscriber) {
    return WaitForCondition([&subscriber]() { return subscriber.getNumPublishers() >= 1; });
}

class MavrosHelperRosTest : public ::testing::Test {
  protected:
    MavrosHelperRosTest()
        : nh_(),
          spinner_(2) {}

    void SetUp() override {
        spinner_.start();

        arm_server_ = nh_.advertiseService(kArmService, &MavrosHelperRosTest::HandleArm, this);
        mode_server_ = nh_.advertiseService(kModeService, &MavrosHelperRosTest::HandleMode, this);

        state_pub_ = nh_.advertise<mavros_msgs::State>(kStateTopic, 1);
        extended_state_pub_ = nh_.advertise<mavros_msgs::ExtendedState>(kExtendedStateTopic, 1);
        sys_status_pub_ = nh_.advertise<mavros_msgs::SysStatus>(kSysStatusTopic, 1);
        battery_pub_ = nh_.advertise<mavros_msgs::BatteryStatus>(kBatteryTopic, 1);
        estimator_pub_ = nh_.advertise<mavros_msgs::EstimatorStatus>(kEstimatorTopic, 1);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(kOdometryTopic, 1);
        target_local_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(kTargetLocalTopic, 1);
        target_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(kTargetAttitudeTopic, 1);

        vision_pose_sub_ =
            nh_.subscribe(kVisionPoseTopic, 1, &MavrosHelperRosTest::HandleVisionPose, this);
        local_setpoint_sub_ = nh_.subscribe(
            kLocalSetpointTopic, 1, &MavrosHelperRosTest::HandleLocalSetpoint, this);
        attitude_setpoint_sub_ = nh_.subscribe(
            kAttitudeSetpointTopic, 1, &MavrosHelperRosTest::HandleAttitudeSetpoint, this);

        helper_.reset(new MavrosHelper(nh_, MavrosHelper_ConfigList(true)));

        ASSERT_TRUE(WaitForSubscriberConnection(state_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(extended_state_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(sys_status_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(battery_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(estimator_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(odom_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(target_local_pub_));
        ASSERT_TRUE(WaitForSubscriberConnection(target_attitude_pub_));

        ASSERT_TRUE(WaitForPublisherConnection(vision_pose_sub_));
        ASSERT_TRUE(WaitForPublisherConnection(local_setpoint_sub_));
        ASSERT_TRUE(WaitForPublisherConnection(attitude_setpoint_sub_));

        ASSERT_TRUE(ros::service::waitForService(kArmService, ros::Duration(2.0)));
        ASSERT_TRUE(ros::service::waitForService(kModeService, ros::Duration(2.0)));
    }

    void TearDown() override {
        helper_.reset();

        vision_pose_sub_.shutdown();
        local_setpoint_sub_.shutdown();
        attitude_setpoint_sub_.shutdown();

        state_pub_.shutdown();
        extended_state_pub_.shutdown();
        sys_status_pub_.shutdown();
        battery_pub_.shutdown();
        estimator_pub_.shutdown();
        odom_pub_.shutdown();
        target_local_pub_.shutdown();
        target_attitude_pub_.shutdown();

        arm_server_.shutdown();
        mode_server_.shutdown();

        spinner_.stop();
    }

    bool HandleArm(mavros_msgs::CommandBool::Request& request,
                   mavros_msgs::CommandBool::Response& response) {
        std::lock_guard<std::mutex> lock(mutex_);
        arm_called_ = true;
        last_arm_value_ = request.value;
        response.success = true;
        return true;
    }

    bool HandleMode(mavros_msgs::SetMode::Request& request,
                    mavros_msgs::SetMode::Response& response) {
        std::lock_guard<std::mutex> lock(mutex_);
        mode_called_ = true;
        last_custom_mode_ = request.custom_mode;
        response.mode_sent = true;
        return true;
    }

    void HandleVisionPose(const geometry_msgs::PoseStamped& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        vision_pose_received_ = true;
        last_vision_pose_ = msg;
    }

    void HandleLocalSetpoint(const mavros_msgs::PositionTarget& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        local_setpoint_received_ = true;
        last_local_setpoint_ = msg;
    }

    void HandleAttitudeSetpoint(const mavros_msgs::AttitudeTarget& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        attitude_setpoint_received_ = true;
        last_attitude_setpoint_ = msg;
    }

    bool ArmCalled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return arm_called_;
    }

    bool ModeCalled() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_called_;
    }

    bool VisionPoseReceived() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return vision_pose_received_;
    }

    bool LocalSetpointReceived() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return local_setpoint_received_;
    }

    bool AttitudeSetpointReceived() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return attitude_setpoint_received_;
    }

    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    std::unique_ptr<MavrosHelper> helper_;

    ros::Publisher state_pub_;
    ros::Publisher extended_state_pub_;
    ros::Publisher sys_status_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher estimator_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher target_local_pub_;
    ros::Publisher target_attitude_pub_;

    ros::Subscriber vision_pose_sub_;
    ros::Subscriber local_setpoint_sub_;
    ros::Subscriber attitude_setpoint_sub_;

    ros::ServiceServer arm_server_;
    ros::ServiceServer mode_server_;

    mutable std::mutex mutex_;
    bool arm_called_ = false;
    bool last_arm_value_ = false;
    bool mode_called_ = false;
    std::string last_custom_mode_;
    bool vision_pose_received_ = false;
    geometry_msgs::PoseStamped last_vision_pose_;
    bool local_setpoint_received_ = false;
    mavros_msgs::PositionTarget last_local_setpoint_;
    bool attitude_setpoint_received_ = false;
    mavros_msgs::AttitudeTarget last_attitude_setpoint_;
};

TEST_F(MavrosHelperRosTest, EndToEndRosBehavior) {
    mavros_msgs::State state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.connected = true;
    state_msg.armed = true;
    state_msg.manual_input = true;
    state_msg.system_status = 4;
    state_msg.mode = mavros_msgs::State::MODE_PX4_OFFBOARD;
    state_pub_.publish(state_msg);

    mavros_msgs::ExtendedState extended_state_msg;
    extended_state_msg.header.stamp = ros::Time::now();
    extended_state_msg.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
    extended_state_pub_.publish(extended_state_msg);

    mavros_msgs::SysStatus sys_status_msg;
    sys_status_msg.header.stamp = ros::Time::now();
    sys_status_msg.load = 760;
    sys_status_msg.voltage_battery = 15500;
    sys_status_msg.current_battery = 1234;
    sys_status_msg.battery_remaining = 87;
    sys_status_pub_.publish(sys_status_msg);

    mavros_msgs::BatteryStatus battery_msg;
    battery_msg.header.stamp = ros::Time::now();
    battery_msg.voltage = 15.2f;
    battery_msg.current = 5.6f;
    battery_msg.remaining = 0.72f;
    battery_pub_.publish(battery_msg);

    mavros_msgs::EstimatorStatus estimator_msg;
    estimator_msg.header.stamp = ros::Time::now();
    estimator_msg.attitude_status_flag = true;
    estimator_msg.velocity_horiz_status_flag = true;
    estimator_msg.pos_horiz_rel_status_flag = true;
    estimator_msg.velocity_vert_status_flag = true;
    estimator_msg.pos_vert_abs_status_flag = true;
    estimator_msg.pos_vert_agl_status_flag = true;
    estimator_msg.pos_horiz_abs_status_flag = true;
    estimator_msg.pred_pos_horiz_abs_status_flag = true;
    estimator_msg.gps_glitch_status_flag = false;
    estimator_msg.accel_error_status_flag = false;
    estimator_pub_.publish(estimator_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = 1.0;
    odom_msg.pose.pose.position.y = -2.0;
    odom_msg.pose.pose.position.z = 3.5;
    odom_msg.pose.pose.orientation.w = 0.9238795325;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.3826834324;
    odom_msg.twist.twist.linear.x = 0.3;
    odom_msg.twist.twist.linear.y = -0.2;
    odom_msg.twist.twist.linear.z = 0.1;
    odom_msg.twist.twist.angular.x = 0.01;
    odom_msg.twist.twist.angular.y = 0.02;
    odom_msg.twist.twist.angular.z = 0.03;
    odom_pub_.publish(odom_msg);

    mavros_msgs::PositionTarget target_local_msg;
    target_local_msg.header.stamp = ros::Time::now();
    target_local_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    target_local_msg.type_mask =
        mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY;
    target_local_msg.position.x = 4.0;
    target_local_msg.position.y = 5.0;
    target_local_msg.position.z = 6.0;
    target_local_msg.velocity.x = 0.7;
    target_local_msg.velocity.y = 0.8;
    target_local_msg.velocity.z = 0.9;
    target_local_msg.acceleration_or_force.x = 1.1;
    target_local_msg.acceleration_or_force.y = 1.2;
    target_local_msg.acceleration_or_force.z = 1.3;
    target_local_msg.yaw = 0.4f;
    target_local_msg.yaw_rate = -0.2f;
    target_local_pub_.publish(target_local_msg);

    ASSERT_TRUE(WaitForCondition([this]() {
        return helper_->get_state().flight_mode == control_common::FlightMode::Offboard &&
               helper_->get_battery().timestamp != ros::Time(0) &&
               helper_->get_estimator_status().timestamp != ros::Time(0) &&
               helper_->get_odometry().timestamp != ros::Time(0) &&
               helper_->get_target_local().timestamp != ros::Time(0);
    }));

    EXPECT_FALSE(helper_->is_ready());

    mavros_msgs::AttitudeTarget target_attitude_msg;
    target_attitude_msg.header.stamp = ros::Time::now();
    target_attitude_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    target_attitude_msg.orientation.w = 0.7071067812;
    target_attitude_msg.orientation.x = 0.0;
    target_attitude_msg.orientation.y = 0.7071067812;
    target_attitude_msg.orientation.z = 0.0;
    target_attitude_msg.body_rate.x = 0.11;
    target_attitude_msg.body_rate.y = 0.12;
    target_attitude_msg.body_rate.z = 0.13;
    target_attitude_msg.thrust = 0.65f;
    target_attitude_pub_.publish(target_attitude_msg);

    ASSERT_TRUE(WaitForCondition(
        [this]() { return helper_->get_target_attitude().timestamp != ros::Time(0); }));

    const control_common::Mavros_State state = helper_->get_state();
    EXPECT_TRUE(state.connected);
    EXPECT_TRUE(state.armed);
    EXPECT_TRUE(state.rc_input);
    EXPECT_EQ(state.system_status, 4);
    EXPECT_EQ(state.flight_mode, control_common::FlightMode::Offboard);
    EXPECT_EQ(state.landed_state, control_common::LandedState::InAir);
    EXPECT_FLOAT_EQ(state.system_load, 0.76f);
    EXPECT_FLOAT_EQ(state.voltage, 15.5f);
    EXPECT_FLOAT_EQ(state.current, 12.34f);
    EXPECT_FLOAT_EQ(state.percent, 0.87f);

    const control_common::Mavros_Battery battery = helper_->get_battery();
    EXPECT_FLOAT_EQ(battery.voltage, 15.2f);
    EXPECT_FLOAT_EQ(battery.current, 5.6f);
    EXPECT_FLOAT_EQ(battery.percent, 0.72f);

    const control_common::Mavros_Estimator estimator = helper_->get_estimator_status();
    EXPECT_TRUE(estimator.attitude_valid);
    EXPECT_TRUE(estimator.local_hroiz_valid);
    EXPECT_TRUE(estimator.local_vertical_valid);
    EXPECT_TRUE(estimator.global_hroiz_valid);
    EXPECT_TRUE(estimator.global_vertical_valid);
    EXPECT_FALSE(estimator.gps_error);
    EXPECT_FALSE(estimator.acc_error);

    const control_common::UAVStateEstimate odom = helper_->get_odometry();
    EXPECT_DOUBLE_EQ(odom.position.x(), 1.0);
    EXPECT_DOUBLE_EQ(odom.position.y(), -2.0);
    EXPECT_DOUBLE_EQ(odom.position.z(), 3.5);
    EXPECT_DOUBLE_EQ(odom.velocity.x(), 0.3);
    EXPECT_DOUBLE_EQ(odom.velocity.y(), -0.2);
    EXPECT_DOUBLE_EQ(odom.velocity.z(), 0.1);
    EXPECT_DOUBLE_EQ(odom.bodyrate.x(), 0.01);
    EXPECT_DOUBLE_EQ(odom.bodyrate.y(), 0.02);
    EXPECT_DOUBLE_EQ(odom.bodyrate.z(), 0.03);

    const Eigen::Quaterniond attitude = helper_->get_attitude_quat();
    EXPECT_TRUE(attitude.coeffs().isApprox(odom.orientation.coeffs(), 1e-9));
    EXPECT_NEAR(helper_->get_yaw_rad(), M_PI / 4.0, 1e-6);
    EXPECT_NEAR(helper_->get_yaw_deg(), 45.0, 1e-6);

    const control_common::Mavros_SetpointLocal target_local = helper_->get_target_local();
    EXPECT_EQ(target_local.frame,
              control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Ned);
    EXPECT_EQ(target_local.mask, target_local_msg.type_mask);
    EXPECT_DOUBLE_EQ(target_local.position.x(), 4.0);
    EXPECT_DOUBLE_EQ(target_local.position.y(), 5.0);
    EXPECT_DOUBLE_EQ(target_local.position.z(), 6.0);
    EXPECT_DOUBLE_EQ(target_local.velocity.x(), 0.7);
    EXPECT_DOUBLE_EQ(target_local.velocity.y(), 0.8);
    EXPECT_DOUBLE_EQ(target_local.velocity.z(), 0.9);
    EXPECT_DOUBLE_EQ(target_local.accel_or_force.x(), 1.1);
    EXPECT_DOUBLE_EQ(target_local.accel_or_force.y(), 1.2);
    EXPECT_DOUBLE_EQ(target_local.accel_or_force.z(), 1.3);
    EXPECT_NEAR(target_local.yaw, 0.4, 1e-6);
    EXPECT_NEAR(target_local.yaw_rate, -0.2, 1e-6);

    const control_common::Mavros_SetpointAttitude target_attitude = helper_->get_target_attitude();
    EXPECT_EQ(target_attitude.mask, target_attitude_msg.type_mask);
    EXPECT_TRUE(target_attitude.attitude.coeffs().isApprox(
        Eigen::Quaterniond(target_attitude_msg.orientation.w,
                           target_attitude_msg.orientation.x,
                           target_attitude_msg.orientation.y,
                           target_attitude_msg.orientation.z)
            .coeffs(),
        1e-9));
    EXPECT_DOUBLE_EQ(target_attitude.body_rate.x(), 0.11);
    EXPECT_DOUBLE_EQ(target_attitude.body_rate.y(), 0.12);
    EXPECT_DOUBLE_EQ(target_attitude.body_rate.z(), 0.13);
    EXPECT_NEAR(target_attitude.thrust, 0.65, 1e-6);

    EXPECT_TRUE(helper_->is_ready());

    ASSERT_TRUE(helper_->set_arm(true));
    ASSERT_TRUE(WaitForCondition([this]() { return ArmCalled(); }));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_TRUE(last_arm_value_);
    }

    ASSERT_TRUE(helper_->set_px4_mode(control_common::FlightMode::Offboard));
    ASSERT_TRUE(WaitForCondition([this]() { return ModeCalled(); }));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_custom_mode_, "OFFBOARD");
    }

    control_common::UAVStateEstimate vision_pose_data;
    vision_pose_data.position = Eigen::Vector3d(7.0, 8.0, 9.0);
    vision_pose_data.orientation = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5);
    ASSERT_TRUE(helper_->pub_vision_pose(vision_pose_data));
    ASSERT_TRUE(WaitForCondition([this]() { return VisionPoseReceived(); }));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.position.x, 7.0);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.position.y, 8.0);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.position.z, 9.0);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.orientation.w, 0.5);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.orientation.x, 0.5);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.orientation.y, 0.5);
        EXPECT_DOUBLE_EQ(last_vision_pose_.pose.orientation.z, 0.5);
    }

    control_common::Mavros_SetpointLocal local_command;
    local_command.frame =
        control_common::Mavros_SetpointLocal::Mavros_LocalFrame::Body_Offset_Ned;
    local_command.mask = control_common::Mavros_SetpointLocal::IgnoreAfz |
                         control_common::Mavros_SetpointLocal::IgnoreYawRate;
    local_command.position = Eigen::Vector3d(-1.0, -2.0, 1.5);
    local_command.velocity = Eigen::Vector3d(0.2, 0.3, 0.4);
    local_command.accel_or_force = Eigen::Vector3d(0.5, 0.6, 0.7);
    local_command.yaw = -0.6;
    local_command.yaw_rate = 0.9;
    ASSERT_TRUE(helper_->pub_local_setpoint(local_command));
    ASSERT_TRUE(WaitForCondition([this]() { return LocalSetpointReceived(); }));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_local_setpoint_.coordinate_frame,
                  mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED);
        EXPECT_EQ(last_local_setpoint_.type_mask, local_command.mask);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.position.x, -1.0);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.position.y, -2.0);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.position.z, 1.5);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.velocity.x, 0.2);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.velocity.y, 0.3);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.velocity.z, 0.4);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.acceleration_or_force.x, 0.5);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.acceleration_or_force.y, 0.6);
        EXPECT_DOUBLE_EQ(last_local_setpoint_.acceleration_or_force.z, 0.7);
        EXPECT_NEAR(last_local_setpoint_.yaw, -0.6, 1e-6);
        EXPECT_NEAR(last_local_setpoint_.yaw_rate, 0.9, 1e-6);
    }

    control_common::Mavros_SetpointAttitude attitude_command;
    attitude_command.mask = control_common::Mavros_SetpointAttitude::IgnoreThrust;
    attitude_command.attitude = Eigen::Quaterniond(0.9238795325, 0.0, 0.3826834324, 0.0);
    attitude_command.body_rate = Eigen::Vector3d(0.21, 0.22, 0.23);
    attitude_command.thrust = 0.55;
    ASSERT_TRUE(helper_->pub_attitude_setpoint(attitude_command));
    ASSERT_TRUE(WaitForCondition([this]() { return AttitudeSetpointReceived(); }));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_attitude_setpoint_.type_mask, attitude_command.mask);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.orientation.w, 0.9238795325);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.orientation.x, 0.0);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.orientation.y, 0.3826834324);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.orientation.z, 0.0);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.body_rate.x, 0.21);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.body_rate.y, 0.22);
        EXPECT_DOUBLE_EQ(last_attitude_setpoint_.body_rate.z, 0.23);
        EXPECT_NEAR(last_attitude_setpoint_.thrust, 0.55, 1e-6);
    }
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_helper_ros_test");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
