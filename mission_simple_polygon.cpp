#include "mission_simple_polygon.h"

#include <spdlog/spdlog.h>

#include "mission_cell.h"
#include "mission_directed_polygon.h"
#include "timer.h"

mission_simple_polygon::mission_simple_polygon(const polygon& poly) {
  OLYSEUS_VERIFY(poly.is_simple());

  timer t;

  for (std::size_t i{0}; i < poly.size(); ++i) {
    const auto& edge{poly.edge(i)};
    const utils::direction dir{edge};
    t.start();
    mission_directed_polygon cell{dir, poly};
    spdlog::info("Edge #{} of {} done in {}ms", i + 1, poly.size(),
                 t.elapsed_ms());
    if (cell.waypoint_count() > 0) {
      cells_.push_back(std::move(cell));
    }
  }

  if (cells_.empty()) {
    min_waypoint_ = 0;
    return;
  }

  min_waypoint_ = std::numeric_limits<std::size_t>::max();
  for (mission_directed_polygon& c : cells_) {
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

mission_simple_polygon::~mission_simple_polygon() = default;

auto mission_simple_polygon::visited() const -> bool { return visited_; }

auto mission_simple_polygon::distance(const point& start) const -> double {
  return closest(start).distance(start);
}

void mission_simple_polygon::build_path(const point& start,
                                        std::vector<point>* result) {
  OLYSEUS_VERIFY(!visited_);
  visited_ = true;

  closest(start).build_path(start, result);
}

std::size_t mission_simple_polygon::waypoint_count() const {
  return min_waypoint_;
}

auto mission_simple_polygon::closest_ptr(const point& start) const
    -> const mission_directed_polygon* {
  OLYSEUS_VERIFY(!cells_.empty());
  double min_dist{std::numeric_limits<double>::max()};
  const mission_directed_polygon* closest_c{nullptr};
  for (const mission_directed_polygon& c : cells_) {
    const double dist{c.distance(start)};
    if (dist < min_dist) {
      min_dist = dist;
      closest_c = &c;
    }
  }
  return closest_c;
}

auto mission_simple_polygon::closest(const point& start) const
    -> const mission_directed_polygon& {
  const mission_directed_polygon* c{closest_ptr(start)};
  OLYSEUS_VERIFY(c != nullptr);
  return *c;
}

auto mission_simple_polygon::closest(const point& start)
    -> mission_directed_polygon& {
  mission_directed_polygon* c{
      const_cast<mission_directed_polygon*>(closest_ptr(start))};
  OLYSEUS_VERIFY(c != nullptr);
  return *c;
}
