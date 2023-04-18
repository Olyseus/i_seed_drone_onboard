#include "waypoint.h"

#include <boost/assert.hpp>  // BOOST_VERIFY

waypoint::waypoint(double latitude, double longitude)
    : latitude_(latitude), longitude_(longitude) {
  BOOST_VERIFY(!mission_altitude_.has_value());
}

waypoint::~waypoint() = default;

// FIXME:
// https://stackoverflow.com/questions/72035989
// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init, hicpp-member-init)
waypoint::waypoint(waypoint&&) noexcept = default;

// NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init, hicpp-member-init)
waypoint::waypoint(const waypoint&) = default;

auto waypoint::is_default_altitude() const -> bool {
  return !mission_altitude_.has_value();
}

void waypoint::set_custom_altitude(float laser_range) {
  BOOST_VERIFY(!is_forward_ready());
  BOOST_VERIFY(!mission_altitude_.has_value());

  // If laser range is good and equals to 'expected_height', the new mission
  // altitude will be the same (no need to change height):
  //   2 * expected_height - expected_height = expected_height
  // If laser range is zero (not possible in practice) it means we are on
  // the ground when the initial altitude is 'expected_height'. Hence
  // we should be 'expected_height' higher, i.e. two 'expected_height'.
  // Bigger laser range means we need use lower mission altitude than before.
  // Smaller laser range means we need use higher mission altitude than before.
  // NOLINTNEXTLINE(*-magic-numbers)
  mission_altitude_ = 2.0F * expected_height - laser_range;
}

auto waypoint::altitude() const -> float {
  return mission_altitude_.value_or(expected_height);
}

void waypoint::set_ready(bool forward) {
  if (forward) {
    BOOST_VERIFY(!is_forward_ready());
    BOOST_VERIFY(!is_backward_ready());
    is_forward_ready_ = true;
  } else {
    BOOST_VERIFY(is_forward_ready());
    BOOST_VERIFY(!is_backward_ready());
    is_backward_ready_ = true;
  }
}

auto waypoint::is_forward_ready() const -> bool { return is_forward_ready_; }

auto waypoint::is_backward_ready() const -> bool { return is_backward_ready_; }

auto waypoint::has_detection() const -> bool {
  BOOST_VERIFY(is_forward_ready());
  return detection_result_.has_value();
}

auto waypoint::heading() const -> float {
  BOOST_VERIFY(has_detection());
  return detection_result_.value().drone_attitude.yaw;
}

void waypoint::save_detection(const detection_result& result) {
  BOOST_VERIFY(is_forward_ready());
  BOOST_VERIFY(!is_backward_ready());

  BOOST_VERIFY(!detection_result_.has_value());
  detection_result_ = result;

  constexpr double eps{1e-4};
  BOOST_VERIFY(std::abs(detection_result_.value().gps.latitude - latitude_) <
               eps);
  BOOST_VERIFY(std::abs(detection_result_.value().gps.longitude - longitude_) <
               eps);

  BOOST_VERIFY(!detection_result_.value().pixels.empty());
}

auto waypoint::get_detection() const -> detection_result {
  BOOST_VERIFY(is_forward_ready());
  BOOST_VERIFY(is_backward_ready());

  BOOST_VERIFY(detection_result_.has_value());
  return detection_result_.value();
}
