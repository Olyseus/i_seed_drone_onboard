#include <gtest/gtest.h>  // TEST_F

#include "polygon_slicer.h"

TEST(polygon_slicer, min_on_edge_max_on_edge_same) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  const utils::kernel_ft x_slice{w / 2};

  utils::point p1{0, 0};
  utils::point p2{x_slice, h};
  utils::point p3{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);

  polygon_slicer slicer{poly};
  auto [y_start, y_stop] = slicer.slice(CGAL::to_double(x_slice));

  int visited{0};
  for (int i{y_start}; i <= y_stop; ++i) {
    ++visited;
  }

  // Invalid range returned, should not be visited
  ASSERT_EQ(visited, 0);
}

TEST(polygon_slicer, min_on_edge_max_on_edge_diff) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  const utils::kernel_ft x_slice{w / 2};

  utils::point p1{0, 0};
  utils::point p2{w, 0};
  utils::point p3{w, 2 * h};
  utils::point p4{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  polygon_slicer slicer{poly};
  auto [y_start, y_stop] = slicer.slice(CGAL::to_double(x_slice));

  ASSERT_EQ(y_start, 0);
  ASSERT_EQ(y_stop, 1);

  int visited{0};
  for (int i{y_start}; i <= y_stop; ++i) {
    ++visited;
  }

  ASSERT_EQ(visited, 2);
}

TEST(polygon_slicer, min_on_edge_max_not_edge) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  const utils::kernel_ft x_slice{w / 2};

  utils::point p1{0, 0};
  utils::point p2{w, 0};
  utils::point p3{w, h};
  utils::point p4{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  polygon_slicer slicer{poly};
  auto [y_start, y_stop] = slicer.slice(CGAL::to_double(x_slice));

  ASSERT_EQ(y_start, 0);
  ASSERT_EQ(y_stop, 1);

  int visited{0};
  for (int i{y_start}; i <= y_stop; ++i) {
    ++visited;
  }

  ASSERT_EQ(visited, 2);
}

TEST(polygon_slicer, min_not_edge_max_on_edge) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  const utils::kernel_ft x_slice{w / 2};

  utils::point p1{0, h};
  utils::point p2{w, 0};
  utils::point p3{w, 2 * h};
  utils::point p4{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  polygon_slicer slicer{poly};
  auto [y_start, y_stop] = slicer.slice(CGAL::to_double(x_slice));

  ASSERT_EQ(y_start, 0);
  ASSERT_EQ(y_stop, 1);

  int visited{0};
  for (int i{y_start}; i <= y_stop; ++i) {
    ++visited;
  }

  ASSERT_EQ(visited, 2);
}

TEST(polygon_slicer, min_not_edge_max_not_edge) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  const utils::kernel_ft x_slice{w / 2};

  utils::point p1{0, h};
  utils::point p2{w, 0};
  utils::point p3{w, h};
  utils::point p4{0, 2 * h};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);
  poly.push_back(p4);

  polygon_slicer slicer{poly};
  auto [y_start, y_stop] = slicer.slice(CGAL::to_double(x_slice));

  ASSERT_EQ(y_start, 0);
  ASSERT_EQ(y_stop, 1);

  int visited{0};
  for (int i{y_start}; i <= y_stop; ++i) {
    ++visited;
  }

  ASSERT_EQ(visited, 2);
}
