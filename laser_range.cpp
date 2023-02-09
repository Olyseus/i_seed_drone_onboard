#include "laser_range.h"

#include <boost/assert.hpp> // BOOST_VERIFY
#include <spdlog/spdlog.h>

laser_range::laser_range() {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  // waypoint 0, bad
  values_.push_back(13.2);
  values_.push_back(15.0);

  // waypoint 1, good
  values_.push_back(15.2);

  // waypoint 2, good
  values_.push_back(14.9);

  // waypoint 3, bad
  values_.push_back(17.8);
  values_.push_back(15.0);

  // backward

  // waypoint 3
  values_.push_back(15.2);

  // waypoint 2
  values_.push_back(14.8);

  // waypoint 1
  values_.push_back(15.1);

  // waypoint 0
  values_.push_back(15.5);
#endif
}

auto laser_range::latest() -> double {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  BOOST_VERIFY(!values_.empty());
  const double result{values_.front()};
  values_.pop_front();
  spdlog::info("Laser range: {}", result);
  return result;
#else
  BOOST_VERIFY(false); // FIXME (implement)
  return 0.0;
#endif
}
