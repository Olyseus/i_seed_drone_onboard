#ifndef MISSION_DIRECTED_CELL_H_
#define MISSION_DIRECTED_CELL_H_

#include "utils.h"

class mission_aligned_cell;

/// \brief Mission cell with a particular direction attached to it
/// \details For each \ref utils::corner of the cell
///   the \ref mission_aligned_cell
///   instance will be created. Each such instance will be checked for the
///   number of waypoints, and only those with a minimal number of waypoints
///   will be kept. When \ref build_path is called we will use closest
///   instance of \ref mission_aligned_cell for path construction
class mission_directed_cell {
 public:
  using direction = utils::direction;
  using polygon = utils::polygon;
  using corner = utils::corner;
  using transformation = utils::transformation;
  using point = utils::point;

  explicit mission_directed_cell(const direction& dir, const polygon& poly);
  ~mission_directed_cell();

  /// \cond private
  mission_directed_cell(const mission_directed_cell&) = delete;
  mission_directed_cell(mission_directed_cell&&) = default;

  mission_directed_cell& operator=(const mission_directed_cell&) = delete;
  mission_directed_cell& operator=(mission_directed_cell&&) = default;
  /// \endcond

  /// \brief The minimum number of waypoints the cell can be covered by
  /// \note Zero number of waypoints can be returned in case if the effective
  ///   cell area is zero, i.e. cell is the result of approximation error
  std::size_t waypoint_count() const;

  /// \brief Distance from \c start to the closest cell
  double distance(const point& start) const;

  /// \brief Build mission path
  /// \param start Starting point of the mission path
  /// \param result vector where mission path will be saved
  void build_path(const point& start, std::vector<point>* result);

 private:
  /// \cond private
  friend class mission_directed_cell_test;
  friend class mission_cell_test;
  /// \endcond

  const mission_aligned_cell* closest_ptr(const point& start_local) const;

  const mission_aligned_cell& closest(const point& start_local) const;
  mission_aligned_cell& closest(const point& start_local);

  transformation forward_;
  transformation back_;
  polygon rotated_polygon_;
  std::size_t min_waypoint_;
  std::vector<mission_aligned_cell> cells_;
};

#endif  // MISSION_DIRECTED_CELL_H_
