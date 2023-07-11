#include <gtest/gtest.h>  // TEST_F

#include "mission_aligned_cell.h"
#include "mission_directed_cell.h"

class mission_directed_cell_test : public ::testing::Test {
 public:
  std::vector<mission_aligned_cell>& cells(mission_directed_cell& c) {
    return c.cells_;
  }
};

TEST_F(mission_directed_cell_test, count) {
  const utils::kernel_ft w{utils::camera_footprint_width_m};
  const utils::kernel_ft h{utils::camera_footprint_height_m};

  utils::point p1{0, h / 2};
  utils::point p2{2 * w, 0};
  utils::point p3{2 * w, 3 * h / 2};

  utils::polygon poly;
  poly.push_back(p1);
  poly.push_back(p2);
  poly.push_back(p3);

  utils::vect v_dir{p2, p3};

  mission_directed_cell m{utils::direction{v_dir}, poly};

  ASSERT_EQ(2, cells(m).size());
  for (const mission_aligned_cell& c : cells(m)) {
    ASSERT_EQ(3, c.waypoint_count());
  }
}
