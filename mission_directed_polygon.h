#ifndef MISSION_DIRECTED_POLYGON_H_
#define MISSION_DIRECTED_POLYGON_H_

#include "utils.h"  // utils::direction

class mission_cell;

class mission_directed_polygon {
 public:
  // Do not use CGAL::Simple_cartesian<double> or CGAL::Cartesian<double>, see:
  // - https://github.com/CGAL/cgal/issues/5218
  using kernel = CGAL::Exact_predicates_exact_constructions_kernel;
  using polygon = CGAL::Polygon_2<kernel>;

  explicit mission_directed_polygon(const utils::direction& dir,
                                    const utils::polygon& polygon);
  ~mission_directed_polygon();

  mission_directed_polygon(const mission_directed_polygon&) = delete;
  mission_directed_polygon(mission_directed_polygon&&) = default;

  mission_directed_polygon& operator=(const mission_directed_polygon&) = delete;
  mission_directed_polygon& operator=(mission_directed_polygon&&) = default;

  std::size_t waypoint_count() const;
  double distance(const utils::point& start) const;
  void build_path(const utils::point& start, std::vector<utils::point>* result);

 private:
  friend class mission_directed_polygon_test;

  const mission_cell* closest(const utils::point& start_local) const;

  utils::polygon rotated_polygon_;  // for test
  utils::transformation forward_;
  utils::transformation back_;

  std::vector<mission_cell> cells_;
  std::size_t waypoint_count_{0};
};

#endif  // MISSION_DIRECTED_POLYGON_H_
