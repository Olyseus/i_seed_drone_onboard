#include "home_altitude.h"

#include <numeric> // std::accumulate

#include <boost/assert.hpp> // BOOST_VERIFY

home_altitude::home_altitude() = default;
home_altitude::~home_altitude() = default;

void home_altitude::mission_start() {
  BOOST_VERIFY(!in_progress_);
  BOOST_VERIFY(altitudes_.empty());

  in_progress_ = true;
}

void home_altitude::set_altitude(double drone_altitude, double mission_altitude) {
  BOOST_VERIFY(in_progress_);

  if (altitudes_.empty()) {
    mission_altitude_ = mission_altitude;
  } else {
    BOOST_VERIFY(mission_altitude_ == mission_altitude);
    const double diff{std::abs(drone_altitude - average_drone_altitude())};
    BOOST_VERIFY(diff < 1.0);
  }

  altitudes_.push_back(drone_altitude);
}

double home_altitude::get_home_altitude() const {
  BOOST_VERIFY(in_progress_);
  BOOST_VERIFY(!altitudes_.empty());

  return average_drone_altitude() - mission_altitude_;
}

void home_altitude::mission_stop() {
  BOOST_VERIFY(in_progress_);

  in_progress_ = false;
  altitudes_.clear();
}

auto home_altitude::average_drone_altitude() const -> double {
  BOOST_VERIFY(in_progress_);
  BOOST_VERIFY(!altitudes_.empty());

  const double s{std::accumulate(altitudes_.begin(), altitudes_.end(), 0.0)};
  return s / altitudes_.size();
}
