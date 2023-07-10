#ifndef MISSION_ALIGNED_CELL_H_
#define MISSION_ALIGNED_CELL_H_

#include "utils.h"

/// \brief A directed cell is transformed so that a whole cell is in the
///   first quadrant (x > 0, y > 0)
/// \details With the help of the \ref polygon_slicer tool we cover
///   cell with waypoints
class mission_aligned_cell {
 public:
  using corner = utils::corner;
  using polygon = utils::polygon;
  using transformation = utils::transformation;
  using point = utils::point;

  explicit mission_aligned_cell(corner c, const polygon& poly);
  ~mission_aligned_cell();

  /// \cond private
  mission_aligned_cell(const mission_aligned_cell&) = delete;
  mission_aligned_cell(mission_aligned_cell&&) = default;

  mission_aligned_cell& operator=(const mission_aligned_cell&) = delete;
  mission_aligned_cell& operator=(mission_aligned_cell&&) = default;
  /// \endcond

  /// \brief The minimum number of waypoints the cell can be covered by
  /// \note Zero number of waypoints can be returned in case if the effective
  ///   cell area is zero, i.e. cell is the result of approximation error
  std::size_t waypoint_count() const;

  /// \brief The distance from point \c start to the closest cell waypoint
  ///   from where path can be constructed
  double distance(const point& start) const;

  /// \brief Build mission path
  /// \param start Starting point of the mission path
  /// \param result vector where mission path will be saved
  point build_path(const point& start, std::vector<point>* result) const;

 private:
  /// \cond private
  friend class mission_aligned_cell_test;
  /// \endcond

  bool fill_column(const std::vector<point>& column, std::vector<point>* result,
                   bool up_to_down) const;

  polygon aligned_polygon_;
  transformation forward_;
  transformation back_;

  std::vector<std::vector<point>> mission_points_;
};

#endif  // MISSION_ALIGNED_CELL_H_
