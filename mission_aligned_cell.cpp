#include "mission_aligned_cell.h"

#include "olyseus_verify.h"  // OLYSEUS_VERIFY
#include "polygon_slicer.h"

mission_aligned_cell::mission_aligned_cell(corner c, const polygon& poly) {
  OLYSEUS_VERIFY(poly.is_simple());
  OLYSEUS_VERIFY(poly.is_convex());
  OLYSEUS_VERIFY(poly.is_counterclockwise_oriented());
  OLYSEUS_VERIFY(poly.size() == 3 || poly.size() == 4);

  utils::transformation reflection;
  const utils::line x_axis{CGAL::ORIGIN, utils::point{1, 0}};
  const utils::line y_axis{CGAL::ORIGIN, utils::point{0, 1}};
  const utils::direction neg_x{-1, 0};

  switch (c) {
    case corner::left_down:
      // (x, y) -> (x, y)
      reflection = CGAL::IDENTITY;
      break;
    case corner::left_up:
      // (x, y) -> (x, -y)
      reflection = utils::transformation(CGAL::REFLECTION, x_axis);
      break;
    case corner::right_down:
      // (x, y) -> (-x, y)
      reflection = utils::transformation(CGAL::REFLECTION, y_axis);
      break;
    case corner::right_up:
      // (x, y) -> (-x, -y)
      reflection = utils::transformation(CGAL::ROTATION, neg_x, 1.0, 1e3);
      break;
    default:
      OLYSEUS_VERIFY(false);
      break;
  }

  const polygon reflected_polygon{CGAL::transform(reflection, poly)};
  const utils::bbox bbox{reflected_polygon.bbox()};
  const utils::transformation translation{
      CGAL::TRANSLATION, utils::vect{-bbox.xmin(), -bbox.ymin()}};

  aligned_polygon_ = CGAL::transform(translation, reflected_polygon);
  if (!aligned_polygon_.is_counterclockwise_oriented()) {
    aligned_polygon_.reverse_orientation();
  }
  OLYSEUS_VERIFY(aligned_polygon_.is_counterclockwise_oriented());
  for (const auto& v : aligned_polygon_.vertices()) {
    OLYSEUS_VERIFY(v.x() >= 0.0);
    OLYSEUS_VERIFY(v.y() >= 0.0);
  }

  forward_ = translation * reflection;
  back_ = forward_.inverse();

  const double w{CGAL::to_double(utils::camera_footprint_width_m)};
  const double h{CGAL::to_double(utils::camera_footprint_height_m)};

  const double x_max_d{aligned_polygon_.bbox().xmax() / w};
  OLYSEUS_VERIFY(x_max_d >= 0.0);
  const auto x_max{static_cast<int>(x_max_d)};
  OLYSEUS_VERIFY(x_max >= 0);

  const double y_max_d{aligned_polygon_.bbox().ymax() / h};
  OLYSEUS_VERIFY(y_max_d >= 0.0);
  const auto y_max{static_cast<int>(y_max_d)};
  OLYSEUS_VERIFY(y_max >= 0);

  const polygon_slicer slicer{aligned_polygon_};

  for (int x{0}; x <= x_max; ++x) {
    const double x_slice{x * w + w / 2};
    auto [y_start, y_stop] = slicer.slice(x_slice);
    std::vector<point> column;
    for (int y{y_start}; y <= y_stop; ++y) {
      const double y_mid{y * h + h / 2};
      column.push_back({x_slice, y_mid});
    }
    if (!column.empty()) {
      mission_points_.push_back(column);
    }
  }
}

mission_aligned_cell::~mission_aligned_cell() = default;

auto mission_aligned_cell::waypoint_count() const -> std::size_t {
  std::size_t count{0};
  for (const auto& v : mission_points_) {
    OLYSEUS_VERIFY(v.size() > 0);
    count += v.size();
  }
  return count;
}

double mission_aligned_cell::distance(const point& start) const {
  OLYSEUS_VERIFY(!mission_points_.empty());
  const point local_start{forward_.transform(start)};

  std::vector<point> points;
  points.push_back(mission_points_.front().front());
  points.push_back(mission_points_.front().back());
  points.push_back(mission_points_.back().front());
  points.push_back(mission_points_.back().back());

  utils::kernel_ft min_dist{std::numeric_limits<double>::max()};
  for (const point& p : points) {
    const utils::kernel_ft dist{CGAL::squared_distance(p, local_start)};
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  return CGAL::to_double(min_dist);
}

auto mission_aligned_cell::build_path(const point& start,
                                      std::vector<point>* result) const
    -> point {
  OLYSEUS_VERIFY(!mission_points_.empty());
  OLYSEUS_VERIFY(!mission_points_.front().empty());
  OLYSEUS_VERIFY(!mission_points_.back().empty());

  const point front_front{back_.transform(mission_points_.front().front())};
  const utils::kernel_ft dist_front_front{
      CGAL::squared_distance(front_front, start)};

  const point front_back{back_.transform(mission_points_.front().back())};
  const utils::kernel_ft dist_front_back{
      CGAL::squared_distance(front_back, start)};

  const point back_front{back_.transform(mission_points_.back().front())};
  const utils::kernel_ft dist_back_front{
      CGAL::squared_distance(back_front, start)};

  const point back_back{back_.transform(mission_points_.back().back())};
  const utils::kernel_ft dist_back_back{
      CGAL::squared_distance(back_back, start)};

  bool left_to_right{false};
  bool up_to_down{false};

  utils::kernel_ft min_dist{std::numeric_limits<double>::max()};

  if (dist_front_front < min_dist) {
    min_dist = dist_front_front;
    left_to_right = true;
    up_to_down = false;
  }

  if (dist_front_back < min_dist) {
    min_dist = dist_front_back;
    left_to_right = true;
    up_to_down = true;
  }

  if (dist_back_front < min_dist) {
    min_dist = dist_back_front;
    left_to_right = false;
    up_to_down = false;
  }

  if (dist_back_back < min_dist) {
    min_dist = dist_back_back;
    left_to_right = false;
    up_to_down = true;
  }

  if (left_to_right) {
    for (auto it{mission_points_.begin()}; it != mission_points_.end(); ++it) {
      up_to_down = fill_column(*it, result, up_to_down);
    }
  } else {
    for (auto it{mission_points_.rbegin()}; it != mission_points_.rend();
         ++it) {
      up_to_down = fill_column(*it, result, up_to_down);
    }
  }

  return result->back();
}

auto mission_aligned_cell::fill_column(const std::vector<point>& column,
                                       std::vector<point>* result,
                                       bool up_to_down) const -> bool {
  std::vector<point> to_add;

  if (up_to_down) {
    to_add.insert(to_add.end(), column.rbegin(), column.rend());
  } else {
    to_add.insert(to_add.end(), column.begin(), column.end());
  }

  for (point& p : to_add) {
    p = back_.transform(p);
  }

  OLYSEUS_VERIFY(result != nullptr);
  result->insert(result->end(), to_add.begin(), to_add.end());

  // Change order
  return !up_to_down;
}
