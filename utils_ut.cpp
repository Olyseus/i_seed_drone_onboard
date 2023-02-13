#include <gtest/gtest.h>

#include "utils.h"

TEST(utils, no_drone_heading) {
  constexpr double drone_heading_degree{0.0};

  float yaw{-1.0};
  float pitch{-1.0};

  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 1944, drone_heading_degree);
  ASSERT_DOUBLE_EQ(yaw, 0.0);
  ASSERT_DOUBLE_EQ(pitch, -90.0);

  constexpr double deg_eps{2.0};

  // I quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(3500, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, 50.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(4000, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, 46.0, deg_eps);
  ASSERT_NEAR(pitch, -67.0, deg_eps);

  // II quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(800, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0, deg_eps);
  ASSERT_NEAR(pitch, -64.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(1600, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0, deg_eps);
  ASSERT_NEAR(pitch, -75.0, deg_eps);

  // III quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(1600, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(800, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0, deg_eps);
  ASSERT_NEAR(pitch, -65.0, deg_eps);

  // IV quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(3500, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, 125.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(4000, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, 129.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  // X-axis
  std::tie(yaw, pitch) = gimbal_rotation_params(5184, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, 90.0, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(0, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, -90.0, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  // Y-axis
  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 0, drone_heading_degree);
  ASSERT_NEAR(yaw, 0.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 3888, drone_heading_degree);
  ASSERT_NEAR(yaw, 180.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);
}

TEST(utils, drone_heading) {
  constexpr double drone_heading_degree{90.0};

  float yaw{-1.0};
  float pitch{-1.0};

  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 1944, drone_heading_degree);
  ASSERT_DOUBLE_EQ(yaw, 0.0 + drone_heading_degree);
  ASSERT_DOUBLE_EQ(pitch, -90.0);

  constexpr double deg_eps{2.0};

  // I quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(3500, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, 50.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(4000, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, 46.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -67.0, deg_eps);

  // II quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(800, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -64.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(1600, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -75.0, deg_eps);

  // III quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(1600, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(800, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -65.0, deg_eps);

  // IV quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(3500, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, (125.0 + drone_heading_degree) - 360.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(4000, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, (129.0 + drone_heading_degree) - 360.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  // X-axis
  std::tie(yaw, pitch) = gimbal_rotation_params(5184, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, 90.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(0, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, -90.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  // Y-axis
  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 0, drone_heading_degree);
  ASSERT_NEAR(yaw, 0.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 3888, drone_heading_degree);
  ASSERT_NEAR(yaw, (180.0 + drone_heading_degree) - 360.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);
}
