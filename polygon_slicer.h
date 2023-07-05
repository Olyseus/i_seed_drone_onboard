#ifndef POLYGON_SLICER_H_
#define POLYGON_SLICER_H_

#include "utils.h"

class polygon_slicer {
 public:
  using kernel = CGAL::Cartesian<double>;
  using polygon = CGAL::Polygon_2<kernel>;
  using point = kernel::Point_2;
  using traits = CGAL::Arr_segment_traits_2<kernel>;
  using segment = traits::Segment_2;

  explicit polygon_slicer(const utils::polygon& poly);
  ~polygon_slicer();

  polygon_slicer(const polygon_slicer&) = delete;
  polygon_slicer(polygon_slicer&&) = delete;

  polygon_slicer& operator=(const polygon_slicer&) = delete;
  polygon_slicer& operator=(polygon_slicer&&) = delete;

  std::pair<int, int> slice(double x_slice) const;

 private:
  polygon polygon_;
  double ymax_;
  const double h_;
};

#endif  // POLYGON_SLICER_H_
