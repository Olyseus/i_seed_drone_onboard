#ifndef MISSION_BUILDER_H_
#define MISSION_BUILDER_H_

#include <vector>

#include "lat_lon.h"

/// \brief Class for building a mission path
/// \details Steps to build mission path:
///   - Input geodetic coordinates converted to geocentric
///   - Centroid of the points will be used as origin of local cartesian system
///   - (x, y) coordinates of points from local cartesian system will be used
///     in \c mission_polygon
///   - Resulted mission path in local cartesian system will be converted back
///     to geodetic coordinates
class mission_builder {
 public:
  /// \brief Build mission path based on input points and home location
  /// \param points List of (latitude, longitude) coordinates, input polygon
  ///   provided by user
  /// \param home (latitude, longitude) coordinate of the home location
  /// \return Mission path. List of (latitude, longitude) coordinates of
  ///   waypoints
  static std::vector<lat_lon> make(const std::vector<lat_lon>& points,
                                   const lat_lon& home);
};

#endif  // MISSION_BUILDER_H_
