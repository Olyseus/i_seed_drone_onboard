#ifndef MISSION_ALIGNED_CELL_H_
#define MISSION_ALIGNED_CELL_H_

#include "utils.h"

class mission_aligned_cell {
 public:
  using corner = utils::corner;
  using polygon = utils::polygon;
  using transformation = utils::transformation;
  using point = utils::point;

  explicit mission_aligned_cell(corner c, const polygon& poly);
  ~mission_aligned_cell();

  mission_aligned_cell(const mission_aligned_cell&) = delete;
  mission_aligned_cell(mission_aligned_cell&&) = default;

  mission_aligned_cell& operator=(const mission_aligned_cell&) = delete;
  mission_aligned_cell& operator=(mission_aligned_cell&&) = default;

  std::size_t waypoint_count() const;
  double distance(const point& start) const;
  point build_path(const point& start, std::vector<point>* result) const;

 private:
  friend class mission_aligned_cell_test;

  bool fill_column(const std::vector<point>& column, std::vector<point>* result,
                   bool up_to_down) const;

  polygon aligned_polygon_;
  transformation forward_;
  transformation back_;

  std::vector<std::vector<point>> mission_points_;
};

#endif  // MISSION_ALIGNED_CELL_H_
