#include "polygon_slicer.h"

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

polygon_slicer::polygon_slicer(const utils::polygon& poly)
    : h_(CGAL::to_double(utils::camera_footprint_height_m)) {
  OLYSEUS_VERIFY(poly.is_counterclockwise_oriented());

  for (const auto& v : poly.vertices()) {
    const point p{CGAL::to_double(v.x()), CGAL::to_double(v.y())};
    polygon_.push_back(p);
  }
  OLYSEUS_VERIFY(polygon_.is_counterclockwise_oriented());

  const utils::bbox bbox{polygon_.bbox()};

  constexpr double eps{1e-3};
  OLYSEUS_VERIFY(bbox.ymin() >= 0 && bbox.ymin() < eps);

  ymax_ = bbox.ymax();
  OLYSEUS_VERIFY(ymax_ > 0);
}

polygon_slicer::~polygon_slicer() = default;

auto polygon_slicer::slice(double x_slice) const -> std::pair<int, int> {
  const std::pair<int, int> invalid_range{1, 0};

  OLYSEUS_VERIFY(x_slice > 0);
  const segment vert_slice{point{x_slice, 0}, point{x_slice, ymax_}};

  std::vector<point> intersect_points;

  for (const auto& edge : polygon_.edges()) {
    const segment s{edge};
    const auto intersect{CGAL::intersection(vert_slice, s)};
    if (!intersect) {
      continue;
    }

    const point* p_intersect{boost::get<point>(&*intersect)};
    if (p_intersect != nullptr) {
      intersect_points.push_back(*p_intersect);
    }

    const segment* vert_intersect{boost::get<segment>(&*intersect)};
    if (vert_intersect != nullptr) {
      intersect_points.push_back(vert_intersect->source());
      intersect_points.push_back(vert_intersect->target());
    }
  }

  double result_max{std::numeric_limits<double>::min()};
  double result_min{std::numeric_limits<double>::max()};

  constexpr double eps{1e-3};

  for (const point& p : intersect_points) {
    OLYSEUS_VERIFY(CGAL::abs(p.x() - x_slice) < eps);
    const double y_curr{CGAL::to_double(p.y())};
    OLYSEUS_VERIFY(y_curr >= 0);
    OLYSEUS_VERIFY(y_curr <= ymax_);
    if (y_curr > result_max) {
      result_max = y_curr;
    }
    if (y_curr < result_min) {
      result_min = y_curr;
    }
  }

  if (result_min > result_max) {
    OLYSEUS_VERIFY(intersect_points.empty());
    return invalid_range;
  }

  OLYSEUS_VERIFY(result_min >= 0);
  OLYSEUS_VERIFY(result_max >= 0);

  OLYSEUS_VERIFY(result_min <= ymax_);
  OLYSEUS_VERIFY(result_max <= ymax_);

  auto result_max_int{static_cast<int>(result_max / h_)};
  OLYSEUS_VERIFY(result_max_int >= 0);

  const auto result_min_int{static_cast<int>(result_min / h_)};
  OLYSEUS_VERIFY(result_min_int >= 0);

  OLYSEUS_VERIFY(result_min_int <= result_max_int);

  const bool min_on_edge{std::abs(result_min_int * h_ - result_min) < eps};
  const bool max_on_edge{std::abs(result_max_int * h_ - result_max) < eps};

  if (min_on_edge && max_on_edge && result_min_int == result_max_int) {
    return invalid_range;
  }

  if (max_on_edge) {
    --result_max_int;
  }

  OLYSEUS_VERIFY(result_min_int <= result_max_int);
  return {result_min_int, result_max_int};
}
