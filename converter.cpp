#include "converter.h"

#include <spdlog/spdlog.h>

#include "olyseus_verify.h"  // OLYSEUS_VERIFY
#include "utils.h"           // deg2rad

auto converter::run(const gps_coordinates& gps, const attitude& drone_attitude,
                    const attitude& gimbal_attitude, float length)
    -> converter_result {
  const Eigen::Vector3d drone_ned_v{
      camera_to_drone_ned(gimbal_attitude, length)};

  // FIXME (real values:
  // https://github.com/Olyseus/i_seed_drone_onboard/issues/10)
  const Eigen::Vector3d drone_ned_p{0.4, 0.0, 0.1};

  const Eigen::Vector3d local_ned_v{
      drone_ned_to_local_ned(drone_attitude, drone_ned_v)};
  const Eigen::Vector3d local_ned_p{
      drone_ned_to_local_ned(drone_attitude, drone_ned_p)};
  const Eigen::Vector3d local_ned_down{0.0, 0.0, 1.0};

  constexpr double eps{1e-3};
  OLYSEUS_VERIFY(local_ned_down.dot(local_ned_v.normalized()) > eps);

  const GeographicLib::LocalCartesian local_cartesian(
      gps.latitude, gps.longitude, gps.altitude);
  const Eigen::Vector3d drone{
      local_ned_to_ecef(local_cartesian, Eigen::Vector3d{0.0, 0.0, 0.0})};

  converter_result result;
  result.p = local_ned_to_ecef(local_cartesian, local_ned_p);
  result.v = local_ned_to_ecef(local_cartesian, local_ned_v) - drone;
  result.d = local_ned_to_ecef(local_cartesian, local_ned_down) - drone;

  constexpr double sanity_dist{500.0};
  constexpr double sanity_norm{2.0};
  OLYSEUS_VERIFY(std::abs(result.d.norm() - 1.0) < eps);
  OLYSEUS_VERIFY(result.v.norm() < sanity_dist);
  OLYSEUS_VERIFY(result.d.dot(result.v.normalized()) > eps);
  OLYSEUS_VERIFY((drone - result.p).norm() < sanity_norm);

  const Eigen::Vector3d p_down(result.p + result.d);

  spdlog::info("{} {} {} 255 0 0", drone(0), drone(1), drone(2));
  spdlog::info("{} {} {} 0 0 255", result.p(0), result.p(1), result.p(2));
  spdlog::info("{} {} {} 255 255 0", p_down(0), p_down(1), p_down(2));

  return result;
}

auto converter::local_ned_to_ecef(
    const GeographicLib::LocalCartesian& local_cartesian,
    const Eigen::Vector3d& p) -> Eigen::Vector3d {
  // 'local_cartesian' is ENU East-North-Up, 'p' is NED North-East-Down
  // https://geographiclib.sourceforge.io/2009-03/classGeographicLib_1_1LocalCartesian.html
  // The z axis is normal to the ellipsoid
  // the y axis points due north
  // The plane z = - h0 is tangent to the ellipsoid

  const double x{p(1)};
  const double y{p(0)};
  const double z{-p(2)};

  double lat{0.0};
  double lon{0.0};
  double alt{0.0};
  local_cartesian.Reverse(x, y, z, lat, lon, alt);

  using real = GeographicLib::Math::real;
  real x_ecef{0.0};
  real y_ecef{0.0};
  real z_ecef{0.0};
  GeographicLib::Geocentric::WGS84().Forward(lat, lon, alt, x_ecef, y_ecef,
                                             z_ecef);

  return Eigen::Vector3d{x_ecef, y_ecef, z_ecef};
}

auto converter::camera_to_drone_ned(const attitude& gimbal_attitude,
                                    float length) -> Eigen::Vector3d {
  const Eigen::Vector3d v{1.0 * length, 0.0, 0.0};

  const double roll{gimbal_attitude.roll * deg2rad};
  const double pitch{gimbal_attitude.pitch * deg2rad};
  const double yaw{gimbal_attitude.yaw * deg2rad};

  // https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
  // clang-format off
  Eigen::Matrix3d rotate_yaw;
  rotate_yaw << std::cos(yaw), -std::sin(yaw), 0.0,
                std::sin(yaw),  std::cos(yaw), 0.0,
                          0.0,            0.0, 1.0;

  Eigen::Matrix3d rotate_pitch;
  rotate_pitch << std::cos(pitch), 0.0, std::sin(pitch),
                              0.0, 1.0,             0.0,
                 -std::sin(pitch), 0.0, std::cos(pitch);

  Eigen::Matrix3d rotate_roll;
  rotate_roll << 1.0,            0.0,             0.0,
                 0.0, std::cos(roll), -std::sin(roll),
                 0.0, std::sin(roll),  std::cos(roll);
  // clang-format on

  // Order is yaw, roll, pitch:
  // https://www.dji.com/id/zenmuse-h20-series
  // https://dji-official-fe.djicdn.com/dps/2e876d4a9a58951ed2dd4680c025b002.png
  return rotate_yaw * rotate_roll * rotate_pitch * v;
}

auto converter::drone_ned_to_local_ned(const attitude& drone_attitude,
                                       const Eigen::Vector3d& v)
    -> Eigen::Vector3d {
  const double roll{drone_attitude.roll * deg2rad};
  const double pitch{drone_attitude.pitch * deg2rad};
  const double yaw{drone_attitude.yaw * deg2rad};

  // https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
  // clang-format off
  Eigen::Matrix3d rotate_yaw;
  rotate_yaw << std::cos(yaw), -std::sin(yaw), 0.0,
                std::sin(yaw),  std::cos(yaw), 0.0,
                          0.0,            0.0, 1.0;

  Eigen::Matrix3d rotate_pitch;
  rotate_pitch << std::cos(pitch), 0.0, std::sin(pitch),
                              0.0, 1.0,             0.0,
                 -std::sin(pitch), 0.0, std::cos(pitch);

  Eigen::Matrix3d rotate_roll;
  rotate_roll << 1.0,            0.0,             0.0,
                 0.0, std::cos(roll), -std::sin(roll),
                 0.0, std::sin(roll),  std::cos(roll);
  // clang-format on

  return rotate_yaw * rotate_pitch * rotate_roll * v;
}
