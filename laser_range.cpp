#include "laser_range.h"

#include <boost/assert.hpp> // BOOST_VERIFY
#include <spdlog/spdlog.h>

auto laser_range::latest() -> double {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  double result;
  if (good_) {
    good_ = false;
    result = 15.0;
  } else {
    good_ = true;
    result = dist_(dev_);
  }
  spdlog::info("Laser range: {}", result);
  return result;
#else
  BOOST_VERIFY(false); // FIXME (implement)
  return 0.0;
#endif
}
