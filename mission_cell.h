#ifndef MISSION_CELL_H_
#define MISSION_CELL_H_

#include "utils.h"

class mission_directed_cell;

/// \brief Component of the polygon, trapezoid or triangle
/// \details For each edge of the cell the \ref mission_directed_cell
///   instance will be created. Each such instance will be checked for the
///   number of waypoints, and only those with a minimal number of waypoints
///   will be kept. When \ref build_path is called we will use closest
///   instance of \ref mission_directed_cell for path construction
class mission_cell {
 public:
  using point = utils::point;
  using polygon = utils::polygon;

  explicit mission_cell(const polygon& poly);
  ~mission_cell();

  /// \cond private
  mission_cell(const mission_cell&) = delete;
  mission_cell(mission_cell&&) = default;

  mission_cell& operator=(const mission_cell&) = delete;
  mission_cell& operator=(mission_cell&&) = delete;
  /// \endcond

  /// \brief The minimum number of waypoints the cell can be covered by
  /// \note Zero number of waypoints can be returned in case if the effective
  ///   cell area is zero, i.e. cell is the result of approximation error
  std::size_t waypoint_count() const;

  /// \brief Once the \ref build_path is used the instance is marked as visited
  /// \return \b true \ref build_path is called already
  /// \return \b false \ref build_path is not called yet
  bool visited() const;

  /// \brief The distance from point \c start to the closest cell waypoint
  ///   from where path can be constructed
  double distance(const point& start) const;

  /// \brief Build mission path
  /// \param start Starting point of the mission path
  /// \param result vector where mission path will be saved
  void build_path(const point& start, std::vector<point>* result);

 private:
  /// \cond private
  friend class mission_cell_test;
  /// \endcond

  const mission_directed_cell* closest_ptr(const point& start) const;

  const mission_directed_cell& closest(const point& start) const;
  mission_directed_cell& closest(const point& start);

  std::vector<mission_directed_cell> cells_;
  std::size_t min_waypoint_;
  bool visited_{false};
};

#endif  // MISSION_CELL_H_
