#include "mission_builder.h"

#include <Eigen/Eigenvalues>  // Eigen::SelfAdjointEigenSolver
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "mission_polygon.h"

auto mission_builder::make(const std::vector<lat_lon>& points,
                           const lat_lon& home) -> std::vector<lat_lon> {
  using real = GeographicLib::Math::real;

  const std::size_t size{points.size()};

  if (size == 0) {
    return points;
  }

  // Earth-centered, Earth-fixed Cartesian coordinate system
  std::vector<Eigen::Vector3d> ecef_points;
  ecef_points.reserve(size);

  for (const lat_lon& p : points) {
    real x{0.0};
    real y{0.0};
    real z{0.0};
    constexpr double alt{0.0};
    GeographicLib::Geocentric::WGS84().Forward(p.latitude(), p.longitude(), alt,
                                               x, y, z);
    ecef_points.emplace_back(x, y, z);
  }

  // Calculate centroid
  Eigen::Vector3d centroid{0.0, 0.0, 0.0};
  for (const Eigen::Vector3d& p : ecef_points) {
    centroid.x() += p.x();
    centroid.y() += p.y();
    centroid.z() += p.z();
  }
  centroid.x() /= static_cast<double>(size);
  centroid.y() /= static_cast<double>(size);
  centroid.z() /= static_cast<double>(size);

  real centroid_lat{0.0};
  real centroid_lon{0.0};
  real centroid_h{0.0};
  GeographicLib::Geocentric::WGS84().Reverse(centroid.x(), centroid.y(),
                                             centroid.z(), centroid_lat,
                                             centroid_lon, centroid_h);

  // Local Cartesian coordinate system
  const GeographicLib::LocalCartesian local_cartesian{centroid_lat,
                                                      centroid_lon, 0.0};

  utils::polygon poly;
  for (const lat_lon& p : points) {
    constexpr double h{0.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    local_cartesian.Forward(p.latitude(), p.longitude(), h, x, y, z);
    poly.push_back({x, y});
  }

  constexpr double h{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  local_cartesian.Forward(home.latitude(), home.longitude(), h, x, y, z);
  const utils::point p_home{x, y};
  mission_polygon mission{poly};

  const std::vector<utils::point> xy_result{mission.make(p_home)};

  std::vector<lat_lon> result;
  result.reserve(xy_result.size());

  for (const utils::point& p : xy_result) {
    const double x{CGAL::to_double(p.x())};
    const double y{CGAL::to_double(p.y())};
    constexpr double z{0.0};
    double lat{0.0};
    double lon{0.0};
    double h{0.0};
    local_cartesian.Reverse(x, y, z, lat, lon, h);
    result.emplace_back(lat, lon);
  }

  return result;
}
