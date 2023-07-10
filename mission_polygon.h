#ifndef MISSION_POLYGON_H_
#define MISSION_POLYGON_H_

#include "utils.h"  // utils::polygon

class mission_simple_polygon;

/// \brief Class to build (x, y) mission path based on input polygon and home
/// \details Initial polygon will be decomposed into simple polygons and
///   further processed in \c mission_simple_polygon class. Then output mission
///   path from \c mission_simple_polygon will be combined back into
///   full mission path.
///
///   Example of a polygon showing a self-intersection shape. Here, polygon
///   {A0, A1, A2, A3, A4, A5, A6} will be split into two polygons:
///   {A0, A1, A2, B} and {A3, A4, A5, A6, B}.
/// \image html self_intersection_polygon.jpg width=20%
class mission_polygon {
 public:
  // Do not use CGAL::Simple_cartesian<double> or CGAL::Cartesian<double>, see:
  // - https://github.com/CGAL/cgal/issues/6307
  using kernel = CGAL::Exact_predicates_exact_constructions_kernel;
  using traits = CGAL::Arr_segment_traits_2<kernel>;
  using segment = traits::Segment_2;
  using point = kernel::Point_2;
  using arrangement = CGAL::Arrangement_2<traits>;

  /// \param poly (x, y) input polygon
  explicit mission_polygon(const utils::polygon& poly);
  ~mission_polygon();

  /// \cond private
  mission_polygon(const mission_polygon&) = delete;
  mission_polygon(mission_polygon&&) = delete;

  mission_polygon& operator=(const mission_polygon&) = delete;
  mission_polygon& operator=(mission_polygon&&) = delete;
  /// \endcond

  /// \brief Build mission math
  std::vector<utils::point> make(const utils::point& home);

 private:
  /// \cond private
  friend class mission_polygon_test;
  /// \endcond

  std::vector<mission_simple_polygon> simple_polygons_;
};

#endif  // MISSION_POLYGON_H_
