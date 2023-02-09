#include "waypoint.h"

#include <boost/assert.hpp> // BOOST_VERIFY

waypoint::waypoint(double latitude, double longitude) : latitude_(latitude), longitude_(longitude) {
  BOOST_VERIFY(!mission_altitude_.has_value());
}

waypoint::~waypoint() = default;
waypoint::waypoint(waypoint&&) = default;
waypoint::waypoint(const waypoint&) = default;

auto waypoint::is_default_altitude() const -> bool {
  return !mission_altitude_.has_value();
}

void waypoint::set_custom_altitude(double laser_range) {
  BOOST_VERIFY(!is_ready());
  BOOST_VERIFY(!mission_altitude_.has_value());

  // If laser range is good and equals to 'expected_height', the new mission
  // altitude will be the same (no need to change height):
  //   2 * expected_height - expected_height = expected_height
  // If laser range is zero (not possible in practice) it means we are on
  // the ground when the initial altitude is 'expected_height'. Hence
  // we should be 'expected_height' higher, i.e. two 'expected_height'.
  // Bigger laser range means we need use lower mission altitude than before.
  // Smaller laser range means we need use higher mission altitude than before.
  mission_altitude_ = 2 * expected_height - laser_range;
}

auto waypoint::altitude() const -> double {
  return mission_altitude_.value_or(expected_height);
}

void waypoint::set_ready() {
  BOOST_VERIFY(!is_ready());
  is_ready_ = true;
}

auto waypoint::is_ready() const -> bool {
  return is_ready_;
}

void waypoint::save_detection(const detection_result& result) {
  BOOST_VERIFY(!detection_result_.has_value());
  detection_result_ = result;

  BOOST_VERIFY(std::abs(detection_result_.value().gps.latitude - latitude_) < 1e-4);
  BOOST_VERIFY(std::abs(detection_result_.value().gps.longitude - longitude_) < 1e-4);

  BOOST_VERIFY(!detection_result_.value().pixels.empty());
}
