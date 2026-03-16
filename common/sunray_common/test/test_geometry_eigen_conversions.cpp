#include <gtest/gtest.h>

#include "sunray_common/geometry_eigen_conversions.h"

namespace sunray_common {
namespace {

TEST(GeometryEigenConversionsTest, QuaternionRoundTripPreservesCoefficients) {
  geometry_msgs::Quaternion msg;
  msg.x = 0.1;
  msg.y = -0.2;
  msg.z = 0.3;
  msg.w = 0.9;

  const Eigen::Quaterniond eigen_quat = geometryToEigen(msg);
  const geometry_msgs::Quaternion round_trip = eigenToGeometry(eigen_quat);

  EXPECT_DOUBLE_EQ(eigen_quat.x(), msg.x);
  EXPECT_DOUBLE_EQ(eigen_quat.y(), msg.y);
  EXPECT_DOUBLE_EQ(eigen_quat.z(), msg.z);
  EXPECT_DOUBLE_EQ(eigen_quat.w(), msg.w);
  EXPECT_DOUBLE_EQ(round_trip.x, msg.x);
  EXPECT_DOUBLE_EQ(round_trip.y, msg.y);
  EXPECT_DOUBLE_EQ(round_trip.z, msg.z);
  EXPECT_DOUBLE_EQ(round_trip.w, msg.w);
}

TEST(GeometryEigenConversionsTest, VectorAndPointConversionsPreserveValues) {
  geometry_msgs::Vector3 vector_msg;
  vector_msg.x = 1.0;
  vector_msg.y = -2.0;
  vector_msg.z = 3.5;

  geometry_msgs::Point point_msg;
  point_msg.x = -4.0;
  point_msg.y = 5.5;
  point_msg.z = 6.0;

  const Eigen::Vector3d vector = geometryToEigen(vector_msg);
  const Eigen::Vector3d point = geometryToEigen(point_msg);
  const geometry_msgs::Vector3 round_trip_vector = eigenToGeometry(vector);
  const geometry_msgs::Point point_from_vector = vectorToPoint(round_trip_vector);

  EXPECT_DOUBLE_EQ(vector.x(), vector_msg.x);
  EXPECT_DOUBLE_EQ(vector.y(), vector_msg.y);
  EXPECT_DOUBLE_EQ(vector.z(), vector_msg.z);
  EXPECT_DOUBLE_EQ(point.x(), point_msg.x);
  EXPECT_DOUBLE_EQ(point.y(), point_msg.y);
  EXPECT_DOUBLE_EQ(point.z(), point_msg.z);
  EXPECT_DOUBLE_EQ(point_from_vector.x, vector_msg.x);
  EXPECT_DOUBLE_EQ(point_from_vector.y, vector_msg.y);
  EXPECT_DOUBLE_EQ(point_from_vector.z, vector_msg.z);
}

TEST(GeometryEigenConversionsTest, PoseConversionBuildsExpectedAffineTransform) {
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = 1.0;
  pose_msg.position.y = 2.0;
  pose_msg.position.z = 3.0;
  pose_msg.orientation.x = 0.0;
  pose_msg.orientation.y = 0.0;
  pose_msg.orientation.z = 0.0;
  pose_msg.orientation.w = 1.0;

  const Eigen::Affine3d pose = geometryToEigen(pose_msg);

  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      pose.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

}  // namespace
}  // namespace sunray_common

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
