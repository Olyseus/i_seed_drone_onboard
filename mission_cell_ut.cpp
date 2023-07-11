#include <gtest/gtest.h>  // TEST_F

#include "mission_aligned_cell.h"
#include "mission_cell.h"
#include "mission_directed_cell.h"

class mission_cell_test : public ::testing::Test {
 public:
  const std::vector<mission_directed_cell>& cells(const mission_cell& c) {
    return c.cells_;
  }

  const std::vector<mission_aligned_cell>& cells(
      const mission_directed_cell& c) {
    return c.cells_;
  }
};

TEST_F(mission_cell_test, count) {
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

  mission_cell m{poly};

  ASSERT_EQ(2, cells(m).size());
  for (const mission_directed_cell& c : cells(m)) {
    ASSERT_EQ(2, c.waypoint_count());
  }
}
