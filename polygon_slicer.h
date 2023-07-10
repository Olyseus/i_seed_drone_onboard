#ifndef POLYGON_SLICER_H_
#define POLYGON_SLICER_H_

#include "utils.h"

/// \brief Utility for path creation inside the cell
class polygon_slicer {
 public:
  using kernel = CGAL::Cartesian<double>;
  using polygon = CGAL::Polygon_2<kernel>;
  using point = kernel::Point_2;
  using traits = CGAL::Arr_segment_traits_2<kernel>;
  using segment = traits::Segment_2;

  explicit polygon_slicer(const utils::polygon& poly);
  ~polygon_slicer();

  /// \cond private
  polygon_slicer(const polygon_slicer&) = delete;
  polygon_slicer(polygon_slicer&&) = delete;

  polygon_slicer& operator=(const polygon_slicer&) = delete;
  polygon_slicer& operator=(polygon_slicer&&) = delete;
  /// \endcond

  /// \details Assuming drone will fly vertically on the X = x_slice
  ///   line (parallel to Y-axis), return the start and stop index
  ///   of the waypoint that will cover given X = x_slice line
  /// \return the start and stop indices of waypoints
  /// \note On the image below you can see the work of the method
  ///   applied to three X = x_slice_(1|2|3) vertical lines
  /// \image html polygon_slicer.png width=40%
  std::pair<int, int> slice(double x_slice) const;

 private:
  polygon polygon_;
  double ymax_;
  const double h_;
};

#endif  // POLYGON_SLICER_H_
