#include <cmath>

#include <gtest/gtest.h>
#include <ros/time.h>

#include <radxa/control/complementary_ahrs.h>

TEST(RadxaAttitudeEstimatorTest, StationaryLevelSampleStaysFinite)
{
  ros::Time::init();
  RadxaComplementaryAHRS estimator;
  const radxa_ap::Vector3f zero_gyro(0.0F, 0.0F, 0.0F);
  const radxa_ap::Vector3f gravity(0.0F, 0.0F, 9.8F);
  const radxa_ap::Vector3f magnetic_north(1.0F, 0.0F, 0.0F);

  for (int i = 0; i < 20; ++i) {
    estimator.update(zero_gyro, gravity, magnetic_north);
  }

  radxa_ap::Matrix3f rotation = estimator.getRotation();
  const radxa_ap::Quaternion quaternion = estimator.getQuaternion();
  EXPECT_FALSE(rotation.is_nan());
  EXPECT_TRUE(std::isfinite(quaternion.q1));
  EXPECT_TRUE(std::isfinite(quaternion.q2));
  EXPECT_TRUE(std::isfinite(quaternion.q3));
  EXPECT_TRUE(std::isfinite(quaternion.q4));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
