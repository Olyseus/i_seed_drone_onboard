#ifndef MISSION_POLYGON_H_
#define MISSION_POLYGON_H_

#include "utils.h"  // utils::polygon

class mission_simple_polygon;

class mission_polygon {
 public:
  // Do not use CGAL::Simple_cartesian<double> or CGAL::Cartesian<double>, see:
  // - https://github.com/CGAL/cgal/issues/6307
  using kernel = CGAL::Exact_predicates_exact_constructions_kernel;
  using traits = CGAL::Arr_segment_traits_2<kernel>;
  using segment = traits::Segment_2;
  using point = kernel::Point_2;
  using arrangement = CGAL::Arrangement_2<traits>;

  explicit mission_polygon(const utils::polygon& poly);
  ~mission_polygon();

  mission_polygon(const mission_polygon&) = delete;
  mission_polygon(mission_polygon&&) = delete;

  mission_polygon& operator=(const mission_polygon&) = delete;
  mission_polygon& operator=(mission_polygon&&) = delete;

  std::vector<utils::point> make(const utils::point& home);

 private:
  friend class mission_polygon_test;

  std::vector<mission_simple_polygon> simple_polygons_;
};

#endif  // MISSION_POLYGON_H_
