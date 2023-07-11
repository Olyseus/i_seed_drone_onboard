#include <gtest/gtest.h>

#include "utils.h"

TEST(utils, no_drone_heading) {
  constexpr double drone_heading_degree{0.0};

  float yaw{-1.0};
  float pitch{-1.0};

  std::tie(yaw, pitch) =
      gimbal_rotation_params(2592, 1944, drone_heading_degree);
  ASSERT_DOUBLE_EQ(yaw, 0.0);
  ASSERT_DOUBLE_EQ(pitch, -90.0);

  constexpr double deg_eps{2.0};

  // I quadrant
  std::tie(yaw, pitch) =
      gimbal_rotation_params(3500, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, 50.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(4000, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, 46.0, deg_eps);
  ASSERT_NEAR(pitch, -67.0, deg_eps);

  // II quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(800, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0, deg_eps);
  ASSERT_NEAR(pitch, -64.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(1600, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0, deg_eps);
  ASSERT_NEAR(pitch, -75.0, deg_eps);

  // III quadrant
  std::tie(yaw, pitch) =
      gimbal_rotation_params(1600, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(800, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0, deg_eps);
  ASSERT_NEAR(pitch, -65.0, deg_eps);

  // IV quadrant
  std::tie(yaw, pitch) =
      gimbal_rotation_params(3500, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, 125.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(4000, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, 129.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  // X-axis
  std::tie(yaw, pitch) =
      gimbal_rotation_params(5184, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, 90.0, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(0, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, -90.0, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  // Y-axis
  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 0, drone_heading_degree);
  ASSERT_NEAR(yaw, 0.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(2592, 3888, drone_heading_degree);
  ASSERT_NEAR(yaw, 180.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);
}

TEST(utils, drone_heading) {
  constexpr double drone_heading_degree{90.0};

  float yaw{-1.0};
  float pitch{-1.0};

  std::tie(yaw, pitch) =
      gimbal_rotation_params(2592, 1944, drone_heading_degree);
  ASSERT_DOUBLE_EQ(yaw, 0.0 + drone_heading_degree);
  ASSERT_DOUBLE_EQ(pitch, -90.0);

  constexpr double deg_eps{2.0};

  // I quadrant
  std::tie(yaw, pitch) =
      gimbal_rotation_params(3500, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, 50.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(4000, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, 46.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -67.0, deg_eps);

  // II quadrant
  std::tie(yaw, pitch) = gimbal_rotation_params(800, 600, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -64.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(1600, 1200, drone_heading_degree);
  ASSERT_NEAR(yaw, -53.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -75.0, deg_eps);

  // III quadrant
  std::tie(yaw, pitch) =
      gimbal_rotation_params(1600, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(800, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, -123.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -65.0, deg_eps);

  // IV quadrant
  std::tie(yaw, pitch) =
      gimbal_rotation_params(3500, 2600, drone_heading_degree);
  ASSERT_NEAR(yaw, (125.0 + drone_heading_degree) - 360.0, deg_eps);
  ASSERT_NEAR(pitch, -76.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(4000, 3100, drone_heading_degree);
  ASSERT_NEAR(yaw, (129.0 + drone_heading_degree) - 360.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  // X-axis
  std::tie(yaw, pitch) =
      gimbal_rotation_params(5184, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, 90.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  std::tie(yaw, pitch) = gimbal_rotation_params(0, 1944, drone_heading_degree);
  ASSERT_NEAR(yaw, -90.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -61.0, deg_eps);

  // Y-axis
  std::tie(yaw, pitch) = gimbal_rotation_params(2592, 0, drone_heading_degree);
  ASSERT_NEAR(yaw, 0.0 + drone_heading_degree, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);

  std::tie(yaw, pitch) =
      gimbal_rotation_params(2592, 3888, drone_heading_degree);
  ASSERT_NEAR(yaw, (180.0 + drone_heading_degree) - 360.0, deg_eps);
  ASSERT_NEAR(pitch, -69.0, deg_eps);
}

TEST(utils, size) {
  const utils::kernel_ft width_scale{4};
  const utils::kernel_ft height_scale{3};
  const utils::kernel_ft ratio{width_scale / height_scale};

  constexpr double eps{1e-3};
  ASSERT_LT(CGAL::abs(ratio -
                      utils::sensor_size_width_m / utils::sensor_size_height_m),
            eps);

  ASSERT_LT(CGAL::abs(ratio - utils::camera_footprint_width_m /
                                  utils::camera_footprint_height_m),
            eps);

  std::cout << "footprint width: "
            << CGAL::to_double(utils::camera_footprint_width_m)
            << ", height: " << CGAL::to_double(utils::camera_footprint_height_m)
            << std::endl;
}

TEST(utils, gimbal_rotation_heading_zero) {
  constexpr double heading_degree{0.0};

  double x_pixel{h20_img_width / 2.0};
  double y_pixel{h20_img_height / 2.0};
  auto [yaw_degree, pitch_degree] =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, 0.0, 1e-3);  // not playing any role
  ASSERT_NEAR(pitch_degree, -90.0, 1e-3);

  double expected_pitch{10.0};
  x_pixel = h20_img_width / 2.0;
  double d_m{std::tan(expected_pitch * M_PI / 180.0) * utils::focal_length_m};
  ASSERT_LT(d_m, utils::sensor_size_height_m / 2.0);
  y_pixel = h20_img_height / 2.0 -
            d_m / utils::sensor_size_height_m * h20_img_height;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, 0.0, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);

  y_pixel = h20_img_height / 2.0 +
            d_m / utils::sensor_size_height_m * h20_img_height;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, 180.0, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);

  expected_pitch = 15.0;
  y_pixel = h20_img_height / 2.0;
  d_m = std::tan(expected_pitch * M_PI / 180.0) * utils::focal_length_m;
  ASSERT_LT(d_m, utils::sensor_size_width_m / 2.0);
  x_pixel = h20_img_width / 2.0 +
            d_m / utils::sensor_size_width_m * h20_img_width;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, 90.0, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);

  x_pixel = h20_img_width / 2.0 -
            d_m / utils::sensor_size_width_m * h20_img_width;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, -90.0, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);
}

TEST(utils, gimbal_rotation_heading) {
  double heading_degree{-45.0};

  double x_pixel{h20_img_width / 2.0};
  double y_pixel{h20_img_height / 2.0};
  auto [yaw_degree, pitch_degree] =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, -45.0, 1e-3);  // not playing any role
  ASSERT_NEAR(pitch_degree, -90.0, 1e-3);

  double expected_pitch{10.0};
  x_pixel = h20_img_width / 2.0;
  double d_m{std::tan(expected_pitch * M_PI / 180.0) * utils::focal_length_m};
  ASSERT_LT(d_m, utils::sensor_size_height_m / 2.0);
  y_pixel = h20_img_height / 2.0 -
            d_m / utils::sensor_size_height_m * h20_img_height;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, heading_degree, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);

  y_pixel = h20_img_height / 2.0 +
            d_m / utils::sensor_size_height_m * h20_img_height;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, 180.0 + heading_degree, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);

  expected_pitch = 15.0;
  y_pixel = h20_img_height / 2.0;
  d_m = std::tan(expected_pitch * M_PI / 180.0) * utils::focal_length_m;
  ASSERT_LT(d_m, utils::sensor_size_width_m / 2.0);
  x_pixel = h20_img_width / 2.0 +
            d_m / utils::sensor_size_width_m * h20_img_width;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, 90.0 + heading_degree, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);

  x_pixel = h20_img_width / 2.0 -
            d_m / utils::sensor_size_width_m * h20_img_width;
  std::tie(yaw_degree, pitch_degree) =
      gimbal_rotation_params(x_pixel, y_pixel, heading_degree);
  ASSERT_NEAR(yaw_degree, -90.0 + heading_degree, 1e-3);
  ASSERT_NEAR(pitch_degree, -90.0 + expected_pitch, 1e-3);
}
