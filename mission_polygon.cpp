#include "mission_polygon.h"

#include <spdlog/spdlog.h>

#include <vector>

#include "mission_directed_polygon.h"
#include "mission_simple_polygon.h"
#include "timer.h"

mission_polygon::mission_polygon(const utils::polygon& poly) {
  // Create simple polygons: https://stackoverflow.com/a/71730804

  spdlog::info("Mission polygon:");
  for (const utils::point& p : poly.vertices()) {
    spdlog::info("  {} {}", CGAL::to_double(p.x()), CGAL::to_double(p.y()));
  }

  std::vector<segment> segments;
  segments.reserve(poly.size());

  OLYSEUS_VERIFY(poly.size() >= 3);
  for (std::size_t i{0}; i < poly.size(); ++i) {
    const utils::point p0_d{poly.vertex(i)};
    const point p0{p0_d.x(), p0_d.y()};
    std::size_t next{i + 1};
    if (next >= poly.size()) {
      next = 0;
    }
    const utils::point p1_d{poly.vertex(next)};
    const point p1{p1_d.x(), p1_d.y()};
    segments.emplace_back(p0, p1);
  }

  arrangement arr;
  CGAL::insert(arr, segments.cbegin(), segments.cend());

  for (auto it = arr.faces_begin(); it != arr.faces_end(); ++it) {
    if (it->is_unbounded()) {
      continue;
    }
    utils::polygon poly;
    auto const begin = it->outer_ccb();
    auto curr = begin;
    do {
      const point p{curr->source()->point()};
      poly.push_back({CGAL::to_double(p.x()), CGAL::to_double(p.y())});
    } while (++curr != begin);
    simple_polygons_.emplace_back(poly);
  }
}

mission_polygon::~mission_polygon() = default;

auto mission_polygon::make(const utils::point& home)
    -> std::vector<utils::point> {
  spdlog::info("Make from point {} {}", CGAL::to_double(home.x()),
               CGAL::to_double(home.y()));

  utils::point start{home};
  std::vector<utils::point> result;

  while (true) {
    double min_distance{std::numeric_limits<double>::max()};
    mission_simple_polygon* closest_poly{nullptr};
    for (mission_simple_polygon& poly : simple_polygons_) {
      if (poly.visited()) {
        continue;
      }
      if (poly.waypoint_count() == 0) {
        continue;
      }
      OLYSEUS_VERIFY(poly.waypoint_count() > 0);
      const double dist{poly.distance(start)};
      if (dist < min_distance) {
        min_distance = dist;
        closest_poly = &poly;
      }
    }
    if (closest_poly == nullptr) {
      break;
    }

    const std::size_t n_before{result.size()};
    closest_poly->build_path(start, &result);
    const std::size_t n_after{result.size()};
    OLYSEUS_VERIFY(n_after >= n_before);
    start = result.back();
  }

  if (result.empty()) {
    spdlog::critical("Invalid empty polygon, no waypoints returned");
    return result;
  }

  std::vector<bool> removed(result.size(), false);
  OLYSEUS_VERIFY(removed.size() == result.size());

  constexpr double eps{1e-1};
  const utils::kernel_ft min_dist(utils::camera_footprint_height_m - eps);
  const utils::kernel_ft min_dist_squared{min_dist * min_dist};

  spdlog::info("Removing points that are close to each other");

  timer t;
  t.start();

  for (std::size_t i{0}; i < result.size(); ++i) {
    if (removed[i]) {
      continue;
    }
    const utils::point& p{result[i]};
    for (std::size_t j{i + 1}; j < result.size(); ++j) {
      if (removed[j]) {
        continue;
      }
      const utils::point& q{result[j]};
      if (CGAL::squared_distance(p, q) < min_dist_squared) {
        removed[j] = true;
      }
    }
  }

  std::size_t count{0};
  for (std::size_t i{result.size()}; i > 0; --i) {
    const std::size_t index{i - 1};
    if (removed[index]) {
      result.erase(result.begin() + index);
      ++count;
    }
  }

  spdlog::info("{} points removed in {} ms", count, t.elapsed_ms());

  return result;
}
