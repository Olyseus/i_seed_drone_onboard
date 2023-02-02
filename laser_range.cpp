#include "laser_range.h"

#include <boost/assert.hpp> // BOOST_VERIFY

auto laser_range::latest() -> double {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  return dist_(dev_);
#else
  BOOST_VERIFY(false); // FIXME (implement)
  return 0.0;
#endif
}
