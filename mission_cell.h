#ifndef MISSION_CELL_H_
#define MISSION_CELL_H_

#include "utils.h"

class mission_directed_cell;

class mission_cell {
 public:
  using point = utils::point;
  using polygon = utils::polygon;

  explicit mission_cell(const polygon& poly);
  ~mission_cell();

  mission_cell(const mission_cell&) = delete;
  mission_cell(mission_cell&&) = default;

  mission_cell& operator=(const mission_cell&) = delete;
  mission_cell& operator=(mission_cell&&) = delete;

  std::size_t waypoint_count() const;
  bool visited() const;
  double distance(const point& start) const;
  void build_path(const point& start, std::vector<point>* result);

 private:
  friend class mission_cell_test;

  const mission_directed_cell* closest_ptr(const point& start) const;

  const mission_directed_cell& closest(const point& start) const;
  mission_directed_cell& closest(const point& start);

  std::vector<mission_directed_cell> cells_;
  std::size_t min_waypoint_;
  bool visited_{false};
};

#endif  // MISSION_CELL_H_
