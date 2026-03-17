#include <gtest/gtest.h>

#include "sunray_common/quad_state_estimate.h"

namespace sunray_common {
namespace {

TEST(QuadStateEstimateTest, DefaultConstructedStateStartsInvalid) {
  const QuadStateEstimate state;

  EXPECT_EQ(state.coordinate_frame, QuadStateEstimate::CoordinateFrame::INVALID);
  EXPECT_FALSE(state.isValid());
  EXPECT_TRUE(state.position.isZero());
  EXPECT_TRUE(state.velocity.isZero());
  EXPECT_TRUE(state.bodyrates.isZero());
  EXPECT_TRUE(state.orientation.coeffs().isApprox(
      Eigen::Quaterniond::Identity().coeffs()));
}

TEST(QuadStateEstimateTest, ConstructFromOdometryPopulatesFields) {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time(12.5);
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = -2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 4.0;
  odom.twist.twist.linear.y = 5.0;
  odom.twist.twist.linear.z = 6.0;
  odom.twist.twist.angular.x = 0.1;
  odom.twist.twist.angular.y = 0.2;
  odom.twist.twist.angular.z = 0.3;

  const QuadStateEstimate state(odom);

  EXPECT_EQ(state.timestamp, odom.header.stamp);
  EXPECT_EQ(state.coordinate_frame, QuadStateEstimate::CoordinateFrame::WORLD);
  EXPECT_TRUE(state.position.isApprox(Eigen::Vector3d(1.0, -2.0, 3.0)));
  EXPECT_TRUE(state.velocity.isApprox(Eigen::Vector3d(4.0, 5.0, 6.0)));
  EXPECT_TRUE(state.bodyrates.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(state.orientation.coeffs().isApprox(
      Eigen::Quaterniond::Identity().coeffs()));
  EXPECT_TRUE(state.isValid());
}

TEST(QuadStateEstimateTest, TransformVelocityToWorldFrameUsesOrientation) {
  QuadStateEstimate state;
  state.coordinate_frame = QuadStateEstimate::CoordinateFrame::WORLD;
  state.orientation = Eigen::AngleAxisd(
      M_PI / 2.0, Eigen::Vector3d::UnitZ());
  state.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);

  state.transformVelocityToWorldFrame();

  EXPECT_NEAR(state.velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(state.velocity.y(), 1.0, 1e-9);
  EXPECT_NEAR(state.velocity.z(), 0.0, 1e-9);
}

TEST(QuadStateEstimateTest, GetYawReturnsHeadingFromOrientation) {
  QuadStateEstimate state;
  state.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ()));

  EXPECT_NEAR(state.getYaw(), M_PI / 3.0, 1e-9);
}

TEST(QuadStateEstimateTest, ToRosMessagePreservesStateValues) {
  QuadStateEstimate state;
  state.timestamp = ros::Time(5.0);
  state.coordinate_frame = QuadStateEstimate::CoordinateFrame::VISION;
  state.position = Eigen::Vector3d(7.0, 8.0, 9.0);
  state.velocity = Eigen::Vector3d(-1.0, -2.0, -3.0);
  state.orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  state.bodyrates = Eigen::Vector3d(0.4, 0.5, 0.6);

  const nav_msgs::Odometry msg = state.toRosMessage();

  EXPECT_EQ(msg.header.stamp, ros::Time(5.0));
  EXPECT_EQ(msg.header.frame_id, "vision");
  EXPECT_EQ(msg.child_frame_id, "body");
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.x, 7.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.y, 8.0);
  EXPECT_DOUBLE_EQ(msg.pose.pose.position.z, 9.0);
  EXPECT_DOUBLE_EQ(msg.twist.twist.linear.x, -1.0);
  EXPECT_DOUBLE_EQ(msg.twist.twist.linear.y, -2.0);
  EXPECT_DOUBLE_EQ(msg.twist.twist.linear.z, -3.0);
  EXPECT_DOUBLE_EQ(msg.twist.twist.angular.x, 0.4);
  EXPECT_DOUBLE_EQ(msg.twist.twist.angular.y, 0.5);
  EXPECT_DOUBLE_EQ(msg.twist.twist.angular.z, 0.6);
}

}  // namespace
}  // namespace sunray_common

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
