#include "mission_directed_polygon.h"

#include <CGAL/Polygon_vertical_decomposition_2.h>
#include <spdlog/spdlog.h>

#include "mission_cell.h"
#include "mission_directed_cell.h"
#include "timer.h"

mission_directed_polygon::mission_directed_polygon(const utils::direction& dir,
                                                   const utils::polygon& poly) {
  OLYSEUS_VERIFY(poly.is_simple());
  OLYSEUS_VERIFY(poly.is_counterclockwise_oriented());

  // Rotation that will move (1, 0) to (dir.y, dir.x).
  // This rotation will move (dir.x, dir.y) to (0, 1), i.e. make it vertical
  const utils::direction rot_dir{dir.dy(), dir.dx()};
  constexpr double den{1e9};
  forward_ = utils::transformation{CGAL::ROTATION, rot_dir, 1.0, den};
  back_ = forward_.inverse();

  rotated_polygon_ = CGAL::transform(forward_, poly);

  polygon rotated_polygon_exact;
  for (const auto& v : rotated_polygon_.vertices()) {
    rotated_polygon_exact.push_back({v.x(), v.y()});
  }

  const CGAL::Polygon_vertical_decomposition_2<kernel> decompose;
  std::vector<polygon> trapezoids;
  spdlog::info("Decomposing polygon");
  timer t;
  t.start();
  decompose(rotated_polygon_exact, std::back_inserter(trapezoids));
  spdlog::info("Decomposing done in {} ms, {} pieces", t.elapsed_ms(),
               trapezoids.size());

  constexpr double eps{1e-3};

  for (const polygon& p_exact : trapezoids) {
    utils::polygon p;

    for (const auto& v : p_exact.vertices()) {
      p.push_back({CGAL::to_double(v.x()), CGAL::to_double(v.y())});
    }

    utils::cleanup_collinear(&p);

    if (p.size() < 3) {
      // Polygon degrades into line
      OLYSEUS_VERIFY(p.size() == 0);
      continue;
    }

    OLYSEUS_VERIFY(p.is_simple());
    OLYSEUS_VERIFY(p.is_convex());
    OLYSEUS_VERIFY(p.is_counterclockwise_oriented());

    const auto& vertices{p.vertices()};

    OLYSEUS_VERIFY(vertices.size() == 3 || vertices.size() == 4);

    if (std::abs(CGAL::to_double(p.area())) < eps) {
      continue;
    }

    mission_cell cell{p};
    if (cell.waypoint_count() > 0) {
      cells_.push_back(std::move(cell));
    }
  }

  if (poly.size() == 3) {
    // Initial polygon is triangle, only one cell expected
    OLYSEUS_VERIFY(cells_.size() <= 1);
  }

  waypoint_count_ = 0;
  for (const mission_cell& c : cells_) {
    waypoint_count_ += c.waypoint_count();
  }
}

mission_directed_polygon::~mission_directed_polygon() = default;

auto mission_directed_polygon::waypoint_count() const -> std::size_t {
  return waypoint_count_;
}

auto mission_directed_polygon::distance(const utils::point& start) const
    -> double {
  const utils::point start_local{forward_.transform(start)};
  const mission_cell* c{closest(start_local)};
  OLYSEUS_VERIFY(c != nullptr);
  return c->distance(start_local);
}

void mission_directed_polygon::build_path(const utils::point& start,
                                          std::vector<utils::point>* result) {
  utils::point start_local{forward_.transform(start)};

  std::vector<utils::point> new_points;

  while (true) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    mission_cell* c{const_cast<mission_cell*>(closest(start_local))};
    if (c == nullptr) {
      break;
    }

    const std::size_t n_before{result->size()};
    c->build_path(start_local, &new_points);
    const std::size_t n_after{result->size()};
    OLYSEUS_VERIFY(n_after >= n_before);
    start_local = new_points.back();
  }

  for (utils::point& p : new_points) {
    p = back_.transform(p);
  }

  result->insert(result->end(), new_points.begin(), new_points.end());
}

auto mission_directed_polygon::closest(const utils::point& start_local) const
    -> const mission_cell* {
  OLYSEUS_VERIFY(!cells_.empty());
  const mission_cell* closest_c{nullptr};
  double min_dist{std::numeric_limits<double>::max()};
  for (const mission_cell& c : cells_) {
    if (c.visited()) {
      continue;
    }
    const double dist{c.distance(start_local)};
    if (dist < min_dist) {
      min_dist = dist;
      closest_c = &c;
    }
  }
  return closest_c;
}
