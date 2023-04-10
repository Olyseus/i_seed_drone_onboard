#include <gtest/gtest.h>

#include "converter.h"

class converter_test : public ::testing::Test {
 public:
  static constexpr double eps{1e-10};

  void camera_to_drone_ned(const attitude& gimbal_attitude, double length,
                           const Eigen::Vector3d& expected) {
    const Eigen::Vector3d result{
        converter::camera_to_drone_ned(gimbal_attitude, length)};
    ASSERT_NEAR(result(0), expected(0), eps);
    ASSERT_NEAR(result(1), expected(1), eps);
    ASSERT_NEAR(result(2), expected(2), eps);
  }

  void drone_ned_to_local_ned(const attitude& drone_attitude,
                              const Eigen::Vector3d& p,
                              const Eigen::Vector3d& expected) {
    const Eigen::Vector3d result{
        converter::drone_ned_to_local_ned(drone_attitude, p)};
    ASSERT_NEAR(result(0), expected(0), eps);
    ASSERT_NEAR(result(1), expected(1), eps);
    ASSERT_NEAR(result(2), expected(2), eps);
  }
};

TEST_F(converter_test, camera_to_drone_ned) {
  attitude a;

  a.roll = 0.0;
  a.pitch = 0.0;
  a.yaw = 0.0;
  camera_to_drone_ned(a, 1.0, {1.0, 0.0, 0.0});

  a.roll = 0.0;
  a.pitch = 0.0;
  a.yaw = 90.0;
  camera_to_drone_ned(a, 1.0, {0.0, 1.0, 0.0});

  a.roll = 0.0;
  a.pitch = 0.0;
  a.yaw = -90.0;
  camera_to_drone_ned(a, 1.0, {0.0, -1.0, 0.0});

  a.roll = 0.0;
  a.pitch = 90.0;
  a.yaw = 0.0;
  camera_to_drone_ned(a, 2.0, {0.0, 0.0, -2.0});

  a.roll = 0.0;
  a.pitch = -90.0;
  a.yaw = 0.0;
  camera_to_drone_ned(a, 2.0, {0.0, 0.0, 2.0});

  a.roll = 0.0;
  a.pitch = 90.0;
  a.yaw = 90.0;
  camera_to_drone_ned(a, 3.0, {0.0, 0.0, -3.0});

  a.roll = 0.0;
  a.pitch = -90.0;
  a.yaw = 90.0;
  camera_to_drone_ned(a, 4.0, {0.0, 0.0, 4.0});

  a.roll = 0.0;
  a.pitch = 90.0;
  a.yaw = -90.0;
  camera_to_drone_ned(a, 5.0, {0.0, 0.0, -5.0});

  a.roll = 0.0;
  a.pitch = -90.0;
  a.yaw = -90.0;
  camera_to_drone_ned(a, 6.0, {0.0, 0.0, 6.0});
}

TEST_F(converter_test, drone_ned_to_local_ned) {
  attitude a;

  a.roll = 0.0;
  a.pitch = 0.0;
  a.yaw = 0.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});

  a.roll = 90.0;
  a.pitch = 0.0;
  a.yaw = 0.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0});

  a.roll = 0.0;
  a.pitch = 90.0;
  a.yaw = 0.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

  a.roll = 0.0;
  a.pitch = 0.0;
  a.yaw = 90.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});

  a.roll = 90.0;
  a.pitch = 90.0;
  a.yaw = 0.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {0.0, -1.0, 0.0});

  a.roll = 90.0;
  a.pitch = 0.0;
  a.yaw = 90.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});

  a.roll = 0.0;
  a.pitch = 90.0;
  a.yaw = 90.0;
  drone_ned_to_local_ned(a, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0});
  drone_ned_to_local_ned(a, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0});
  drone_ned_to_local_ned(a, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0});
}
