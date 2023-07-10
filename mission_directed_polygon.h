#ifndef MISSION_DIRECTED_POLYGON_H_
#define MISSION_DIRECTED_POLYGON_H_

#include "utils.h"  // utils::direction

class mission_cell;

/// \brief Mission polygon with a particular direction attached to it
/// \details Polygon will be transformed in a way that attached
///   direction became vertical. Then vertical decomposition algorithm
///   will be applied to split polygon into trapezoids and rectangles,
///   collectively called \ref mission_cell
/// \image html mission_cells.png width=40%
/// Considering the direct polygon above:
/// {A0, A1, A2, A3, A4} and vector {A4, A0},
/// it can be decomposed into cells:
/// {A0, A3', A3, A4}, {A1, A1', A3, A3'} and {A1, A1', A2}
class mission_directed_polygon {
 public:
  // Do not use CGAL::Simple_cartesian<double> or CGAL::Cartesian<double>, see:
  // - https://github.com/CGAL/cgal/issues/5218
  using kernel = CGAL::Exact_predicates_exact_constructions_kernel;
  using polygon = CGAL::Polygon_2<kernel>;

  explicit mission_directed_polygon(const utils::direction& dir,
                                    const utils::polygon& polygon);
  ~mission_directed_polygon();

  /// \cond private
  mission_directed_polygon(const mission_directed_polygon&) = delete;
  mission_directed_polygon(mission_directed_polygon&&) = default;

  mission_directed_polygon& operator=(const mission_directed_polygon&) = delete;
  mission_directed_polygon& operator=(mission_directed_polygon&&) = default;
  /// \endcond

  /// \return Total number of waypoints in all the cells
  std::size_t waypoint_count() const;

  /// \brief Distance from \c start to the closest cell
  double distance(const utils::point& start) const;

  /// \brief Build mission path
  /// \param start Starting point of the mission path
  /// \param result vector where mission path will be saved
  void build_path(const utils::point& start, std::vector<utils::point>* result);

 private:
  /// \cond private
  friend class mission_directed_polygon_test;
  /// \endcond

  const mission_cell* closest(const utils::point& start_local) const;

  utils::polygon rotated_polygon_;  // for test
  utils::transformation forward_;
  utils::transformation back_;

  std::vector<mission_cell> cells_;
  std::size_t waypoint_count_{0};
};

#endif  // MISSION_DIRECTED_POLYGON_H_
