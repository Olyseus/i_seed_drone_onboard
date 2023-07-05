#ifndef MISSION_DIRECTED_CELL_H_
#define MISSION_DIRECTED_CELL_H_

#include "utils.h"

class mission_aligned_cell;

class mission_directed_cell {
 public:
  using direction = utils::direction;
  using polygon = utils::polygon;
  using corner = utils::corner;
  using transformation = utils::transformation;
  using point = utils::point;

  explicit mission_directed_cell(const direction& dir, const polygon& poly);
  ~mission_directed_cell();

  mission_directed_cell(const mission_directed_cell&) = delete;
  mission_directed_cell(mission_directed_cell&&) = default;

  mission_directed_cell& operator=(const mission_directed_cell&) = delete;
  mission_directed_cell& operator=(mission_directed_cell&&) = default;

  std::size_t waypoint_count() const;
  double distance(const point& start) const;
  void build_path(const point& start, std::vector<point>* result);

 private:
  friend class mission_directed_cell_test;
  friend class mission_cell_test;

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
