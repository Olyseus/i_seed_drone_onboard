#include "mission_directed_cell.h"

#include "mission_aligned_cell.h"
#include "olyseus_verify.h"  // OLYSEUS_VERIFY

mission_directed_cell::mission_directed_cell(const direction& dir,
                                             const polygon& poly) {
  OLYSEUS_VERIFY(poly.is_simple());
  OLYSEUS_VERIFY(poly.is_convex());
  OLYSEUS_VERIFY(poly.is_counterclockwise_oriented());

  // Rotation that will move (1, 0) to (dir.y, dir.x).
  // This rotation will move (dir.x, dir.y) to (0, 1), i.e. make it vertical
  const direction rot_dir{dir.dy(), dir.dx()};
  constexpr double den{1e3};
  forward_ = transformation{CGAL::ROTATION, rot_dir, 1.0, den};
  back_ = forward_.inverse();
  rotated_polygon_ = CGAL::transform(forward_, poly);

  for (const corner c : {corner::left_down, corner::left_up, corner::right_down,
                         corner::right_up}) {
    mission_aligned_cell cell{c, rotated_polygon_};
    if (cell.waypoint_count() > 0) {
      cells_.push_back(std::move(cell));
    }
  }

  if (cells_.empty()) {
    min_waypoint_ = 0;
    return;
  }

  min_waypoint_ = std::numeric_limits<std::size_t>::max();
  for (const mission_aligned_cell& c : cells_) {
    const std::size_t count{c.waypoint_count()};
    OLYSEUS_VERIFY(count > 0);
    if (count < min_waypoint_) {
      min_waypoint_ = count;
    }
  }

  for (auto it{cells_.begin()}; it != cells_.end();) {
    if (it->waypoint_count() == min_waypoint_) {
      ++it;
    } else {
      it = cells_.erase(it);
    }
  }

  OLYSEUS_VERIFY(!cells_.empty());
}

auto mission_directed_cell::waypoint_count() const -> std::size_t {
  return min_waypoint_;
}

mission_directed_cell::~mission_directed_cell() = default;

auto mission_directed_cell::distance(const point& start) const -> double {
  const point start_local{forward_.transform(start)};
  return closest(start_local).distance(start_local);
}

void mission_directed_cell::build_path(const point& start,
                                       std::vector<point>* result) {
  const point start_local{forward_.transform(start)};

  std::vector<point> new_points;
  closest(start_local).build_path(start_local, &new_points);
  OLYSEUS_VERIFY(!new_points.empty());

  for (point& p : new_points) {
    p = back_.transform(p);
  }

  OLYSEUS_VERIFY(result);
  result->insert(result->end(), new_points.begin(), new_points.end());
}

auto mission_directed_cell::closest_ptr(const point& start_local) const
    -> const mission_aligned_cell* {
  OLYSEUS_VERIFY(!cells_.empty());
  const mission_aligned_cell* closest_c{nullptr};
  double min_dist{std::numeric_limits<double>::max()};
  for (const mission_aligned_cell& c : cells_) {
    const double dist{c.distance(start_local)};
    if (dist < min_dist) {
      min_dist = dist;
      closest_c = &c;
    }
  }
  return closest_c;
}

auto mission_directed_cell::closest(const point& start_local) const
    -> const mission_aligned_cell& {
  const mission_aligned_cell* c{closest_ptr(start_local)};
  OLYSEUS_VERIFY(c != nullptr);
  return *c;
}

auto mission_directed_cell::closest(const point& start_local)
    -> mission_aligned_cell& {
  mission_aligned_cell* c{
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
      const_cast<mission_aligned_cell*>(closest_ptr(start_local))};
  OLYSEUS_VERIFY(c != nullptr);
  return *c;
}
