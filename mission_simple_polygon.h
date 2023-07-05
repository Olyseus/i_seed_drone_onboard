#ifndef MISSION_SIMPLE_POLYGON_H_
#define MISSION_SIMPLE_POLYGON_H_

#include "utils.h"  // utils::polygon

class mission_directed_polygon;

class mission_simple_polygon {
 public:
  using polygon = utils::polygon;
  using point = utils::point;

  explicit mission_simple_polygon(const polygon& poly);
  ~mission_simple_polygon();

  bool visited() const;
  double distance(const point& start) const;
  void build_path(const point& start, std::vector<point>* result);

  mission_simple_polygon(const mission_simple_polygon&) = delete;
  mission_simple_polygon(mission_simple_polygon&&) = default;

  mission_simple_polygon& operator=(const mission_simple_polygon&) = delete;
  mission_simple_polygon& operator=(mission_simple_polygon&&) = delete;

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
