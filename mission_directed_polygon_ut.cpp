#include <CGAL/Boolean_set_operations_2.h>  // CGAL::symmetric_difference
#include <gtest/gtest.h>                    // TEST_F

#include "mission_cell.h"
#include "mission_directed_polygon.h"

class mission_directed_polygon_test : public ::testing::Test {
 public:
  using polygon = utils::polygon;

  polygon& rotated_polygon(mission_directed_polygon& m) {
    return m.rotated_polygon_;
  }

  const std::vector<mission_cell>& cells(const mission_directed_polygon& m) {
    return m.cells_;
  }

  void verify_equal(polygon& a, polygon& b) {
    if (!a.is_counterclockwise_oriented()) {
      a.reverse_orientation();
    }
    if (!b.is_counterclockwise_oriented()) {
      b.reverse_orientation();
    }
    BOOST_VERIFY(a.is_counterclockwise_oriented());
    BOOST_VERIFY(b.is_counterclockwise_oriented());
    std::vector<utils::polygon_with_holes> result;
    CGAL::symmetric_difference(a, b, std::back_inserter(result));
    ASSERT_EQ(result.size(), 0);
  }
};

TEST_F(mission_directed_polygon_test, no_change) {
  utils::direction dir{0, 1};

  utils::polygon poly;
  poly.push_back({0, 0});
  poly.push_back({1, 0});
  poly.push_back({0, 1});

  mission_directed_polygon dir_poly{dir, poly};
  verify_equal(poly, rotated_polygon(dir_poly));
}

TEST_F(mission_directed_polygon_test, rotate_left) {
  utils::direction dir{1, 0};

  utils::polygon poly;
  poly.push_back({0, 0});
  poly.push_back({1, 0});
  poly.push_back({0, 1});

  utils::polygon expected_poly;
  expected_poly.push_back({0, 0});
  expected_poly.push_back({-1, 0});
  expected_poly.push_back({0, 1});

  mission_directed_polygon dir_poly{dir, poly};
  verify_equal(expected_poly, rotated_polygon(dir_poly));
}

TEST_F(mission_directed_polygon_test, count) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point p1{0, 0};
  utils::point p2{w, 0};
  utils::point p3{w, 2 * h};
  utils::point p4{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  utils::direction dir{0, 1};

  mission_directed_polygon dir_poly{dir, poly};
  ASSERT_EQ(cells(dir_poly).size(), 1);
  ASSERT_EQ(dir_poly.waypoint_count(), 2);
}
