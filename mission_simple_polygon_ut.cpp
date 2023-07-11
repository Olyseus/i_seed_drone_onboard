#include <gtest/gtest.h>  // TEST_F

#include "mission_simple_polygon.h"

class mission_simple_polygon_test : public ::testing::Test {
 public:
};

TEST_F(mission_simple_polygon_test, count) {
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

  mission_simple_polygon simple{poly};
  ASSERT_EQ(simple.waypoint_count(), 2);

  ASSERT_GE(simple.distance(p1), 0.0);  // sanity check
}
