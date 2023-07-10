#ifndef MISSION_SIMPLE_POLYGON_H_
#define MISSION_SIMPLE_POLYGON_H_

#include "utils.h"  // utils::polygon

class mission_directed_polygon;

/// \brief Mission polygon without self-intersections
/// \details For each edge of the polygon the \ref mission_directed_polygon
///   instance will be created. Each such instance will be checked for the
///   number of waypoints, and only those with a minimal number of waypoints
///   will be kept. When \ref build_path is called we will use closest
///   instance of \ref mission_directed_polygon for path construction
/// \image html simple_polygon.png width=20%
/// E.g., the simple polygon {A0, A1, A2, A3, A4} will produce five
/// instances of the directed polygons, shown below by the red arrows.
/// \image html directed_polygons.png width=80%
class mission_simple_polygon {
 public:
  using polygon = utils::polygon;
  using point = utils::point;

  explicit mission_simple_polygon(const polygon& poly);
  ~mission_simple_polygon();

  /// \brief Once the \ref build_path is used the instance is marked as visited
  /// \return \b true \ref build_path is called already
  /// \return \b false \ref build_path is not called yet
  bool visited() const;

  /// \brief The distance from point \c start to the closest polygon waypoint
  ///   from where path can be constructed
  double distance(const point& start) const;

  /// \brief Build mission path
  /// \param start Starting point of the mission path
  /// \param result vector where mission path will be saved
  void build_path(const point& start, std::vector<point>* result);

  /// \cond private
  mission_simple_polygon(const mission_simple_polygon&) = delete;
  mission_simple_polygon(mission_simple_polygon&&) = default;

  mission_simple_polygon& operator=(const mission_simple_polygon&) = delete;
  mission_simple_polygon& operator=(mission_simple_polygon&&) = delete;
  /// \endcond

  /// \brief The minimum number of waypoints the polygon can be covered by
  /// \note Zero number of waypoints can be returned in case if the effective
  ///   polygon area is zero, i.e. polygon is the result of approximation error
  std::size_t waypoint_count() const;

 private:
  bool visited_{false};

  const mission_directed_polygon* closest_ptr(const point& start) const;

  const mission_directed_polygon& closest(const point& start) const;
  mission_directed_polygon& closest(const point& start);

  std::vector<mission_directed_polygon> cells_;
  std::size_t min_waypoint_{0};
};

#endif  //  MISSION_SIMPLE_POLYGON_H_
