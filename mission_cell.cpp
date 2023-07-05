#include "mission_cell.h"

#include "mission_aligned_cell.h"
#include "mission_directed_cell.h"
#include "olyseus_verify.h"  // OLYSEUS_VERIFY

mission_cell::mission_cell(const polygon& poly) {
  OLYSEUS_VERIFY(poly.is_simple());
  OLYSEUS_VERIFY(poly.is_convex());
  OLYSEUS_VERIFY(poly.is_counterclockwise_oriented());

  OLYSEUS_VERIFY(poly.size() == 3 || poly.size() == 4);

  for (const auto& edge : poly.edges()) {
    const utils::direction dir{edge};
    mission_directed_cell cell{dir, poly};
    if (cell.waypoint_count() > 0) {
      cells_.push_back(std::move(cell));
    }
  }

  if (cells_.empty()) {
    min_waypoint_ = 0;
    return;
  }

  min_waypoint_ = std::numeric_limits<std::size_t>::max();
  for (mission_directed_cell& c : cells_) {
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

mission_cell::~mission_cell() = default;

std::size_t mission_cell::waypoint_count() const { return min_waypoint_; }

bool mission_cell::visited() const { return visited_; }

double mission_cell::distance(const point& start) const {
  return closest(start).distance(start);
}

void mission_cell::build_path(const point& start, std::vector<point>* result) {
  OLYSEUS_VERIFY(!visited_);
  visited_ = true;
  closest(start).build_path(start, result);
}

auto mission_cell::closest_ptr(const point& start) const
    -> const mission_directed_cell* {
  OLYSEUS_VERIFY(!cells_.empty());
  const mission_directed_cell* closest_c{nullptr};
  double min_dist{std::numeric_limits<double>::max()};
  for (const mission_directed_cell& c : cells_) {
    const double dist{c.distance(start)};
    if (dist < min_dist) {
      min_dist = dist;
      closest_c = &c;
    }
  }
  return closest_c;
}

auto mission_cell::closest(const point& start) const
    -> const mission_directed_cell& {
  const mission_directed_cell* c{closest_ptr(start)};
  OLYSEUS_VERIFY(c != nullptr);
  return *c;
}

auto mission_cell::closest(const point& start) -> mission_directed_cell& {
  mission_directed_cell* c{
      const_cast<mission_directed_cell*>(closest_ptr(start))};
  OLYSEUS_VERIFY(c != nullptr);
  return *c;
}
