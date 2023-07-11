#include <CGAL/Boolean_set_operations_2.h>  // CGAL::symmetric_difference
#include <gtest/gtest.h>                    // TEST_F

#include "mission_aligned_cell.h"

class mission_aligned_cell_test : public ::testing::Test {
 public:
  using polygon = utils::polygon;
  using point = utils::point;

  polygon& aligned_polygon(mission_aligned_cell& c) {
    return c.aligned_polygon_;
  }

  point transform_back(const point& p, mission_aligned_cell& c) {
    return c.back_.transform(p);
  }

  std::size_t count_cells(const mission_aligned_cell& c) const {
    return c.waypoint_count();
  }

  void verify_path(const mission_aligned_cell& c, const point& start,
                   const std::vector<point>& expected_path) {
    std::vector<point> path;
    c.build_path(start, &path);
    ASSERT_EQ(path.size(), expected_path.size());
    for (std::size_t i{0}; i < path.size(); ++i) {
      constexpr double eps{1e-3};
      ASSERT_LE(CGAL::squared_distance(path[i], expected_path[i]), eps);
    }
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

TEST_F(mission_aligned_cell_test, left_down) {
  utils::point p1{1, 1};
  utils::point p2{2, 3};
  utils::point p3{4, 2};

  utils::polygon poly;
  poly.push_back(p3);
  poly.push_back(p2);
  poly.push_back(p1);

  mission_aligned_cell m{utils::corner::left_down, poly};

  utils::point q1{0, 0};
  utils::point q2{1, 2};
  utils::point q3{3, 1};

  utils::polygon expected_poly;
  expected_poly.push_back(q3);
  expected_poly.push_back(q2);
  expected_poly.push_back(q1);

  verify_equal(expected_poly, aligned_polygon(m));

  ASSERT_EQ(p1, transform_back(q1, m));
  ASSERT_EQ(p2, transform_back(q2, m));
  ASSERT_EQ(p3, transform_back(q3, m));
}

TEST_F(mission_aligned_cell_test, left_up) {
  utils::point p1{1, 1};
  utils::point p2{2, 3};
  utils::point p3{4, 2};

  utils::polygon poly;
  poly.push_back(p3);
  poly.push_back(p2);
  poly.push_back(p1);

  mission_aligned_cell m{utils::corner::left_up, poly};

  utils::point q1{0, 2};
  utils::point q2{1, 0};
  utils::point q3{3, 1};

  utils::polygon expected_poly;
  expected_poly.push_back(q3);
  expected_poly.push_back(q2);
  expected_poly.push_back(q1);

  verify_equal(expected_poly, aligned_polygon(m));

  ASSERT_EQ(p1, transform_back(q1, m));
  ASSERT_EQ(p2, transform_back(q2, m));
  ASSERT_EQ(p3, transform_back(q3, m));
}

TEST_F(mission_aligned_cell_test, right_down) {
  utils::point p1{1, 1};
  utils::point p2{2, 3};
  utils::point p3{4, 2};

  utils::polygon poly;
  poly.push_back(p3);
  poly.push_back(p2);
  poly.push_back(p1);

  mission_aligned_cell m{utils::corner::right_down, poly};

  utils::point q1{3, 0};
  utils::point q2{2, 2};
  utils::point q3{0, 1};

  utils::polygon expected_poly;
  expected_poly.push_back(q3);
  expected_poly.push_back(q2);
  expected_poly.push_back(q1);

  verify_equal(expected_poly, aligned_polygon(m));

  ASSERT_EQ(p1, transform_back(q1, m));
  ASSERT_EQ(p2, transform_back(q2, m));
  ASSERT_EQ(p3, transform_back(q3, m));
}

TEST_F(mission_aligned_cell_test, right_up) {
  utils::point p1{1, 1};
  utils::point p2{2, 3};
  utils::point p3{4, 2};

  utils::polygon poly;
  poly.push_back(p3);
  poly.push_back(p2);
  poly.push_back(p1);

  mission_aligned_cell m{utils::corner::right_up, poly};

  utils::point q1{3, 2};
  utils::point q2{2, 0};
  utils::point q3{0, 1};

  utils::polygon expected_poly;
  expected_poly.push_back(q3);
  expected_poly.push_back(q2);
  expected_poly.push_back(q1);

  verify_equal(expected_poly, aligned_polygon(m));

  ASSERT_EQ(p1, transform_back(q1, m));
  ASSERT_EQ(p2, transform_back(q2, m));
  ASSERT_EQ(p3, transform_back(q3, m));
}

TEST_F(mission_aligned_cell_test, one_cell) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point p1{0, 0};
  utils::point p2{w, 0};
  utils::point p3{w, h};
  utils::point p4{0, h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  mission_aligned_cell m{utils::corner::left_down, poly};
  ASSERT_EQ(1, count_cells(m));
}

TEST_F(mission_aligned_cell_test, two_cells) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point p1{0, 0};
  utils::point p2{1.5 * w, 0};
  utils::point p3{1.5 * w, h};
  utils::point p4{0, h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  mission_aligned_cell m{utils::corner::left_down, poly};
  ASSERT_EQ(2, count_cells(m));
}

TEST_F(mission_aligned_cell_test, three_cells) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point p1{0, 0};
  utils::point p2{2 * w, 0};
  utils::point p3{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);

  mission_aligned_cell m{utils::corner::left_down, poly};
  ASSERT_EQ(3, count_cells(m));
}

TEST_F(mission_aligned_cell_test, var_cells) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point p1{0, h / 2};
  utils::point p2{2 * w, 0};
  utils::point p3{2 * w, 3 * h / 2};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);

  ASSERT_EQ(3,
            count_cells(mission_aligned_cell{utils::corner::left_down, poly}));
  ASSERT_EQ(3,
            count_cells(mission_aligned_cell{utils::corner::right_down, poly}));
  ASSERT_EQ(4, count_cells(mission_aligned_cell{utils::corner::left_up, poly}));
  ASSERT_EQ(4,
            count_cells(mission_aligned_cell{utils::corner::right_up, poly}));
}

TEST_F(mission_aligned_cell_test, mission) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point q1{w / 2, h / 2};
  utils::point q2{w / 2, 3 * h / 2};
  utils::point q3{3 * w / 2, 3 * h / 2};
  utils::point q4{3 * w / 2, h / 2};

  utils::point p1{0, 0};
  utils::point p2{0, 2 * h};
  utils::point p3{2 * w, 2 * h};
  utils::point p4{2 * w, 0};

  utils::polygon poly;
  poly.push_back(p4);
  poly.push_back(p3);
  poly.push_back(p2);
  poly.push_back(p1);

  for (const utils::corner& c :
       {utils::corner::left_down, utils::corner::left_up,
        utils::corner::right_down, utils::corner::right_up}) {
    const mission_aligned_cell m{c, poly};
    verify_path(m, q1, {q1, q2, q3, q4});
    verify_path(m, q2, {q2, q1, q4, q3});
    verify_path(m, q3, {q3, q4, q1, q2});
    verify_path(m, q4, {q4, q3, q2, q1});
  }
}
