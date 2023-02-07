#include "home_altitude.h"

#include <numeric> // std::accumulate

#include <boost/assert.hpp> // BOOST_VERIFY
#include <spdlog/spdlog.h>

home_altitude::home_altitude() = default;
home_altitude::~home_altitude() = default;

void home_altitude::mission_start() {
  BOOST_VERIFY(!in_progress_);
  BOOST_VERIFY(!home_altitude_.has_value());

  in_progress_ = true;
}

void home_altitude::set_altitude(double drone_altitude, double mission_altitude, double home_altitude) {
  BOOST_VERIFY(in_progress_);

  if (home_altitude_.has_value()) {
    BOOST_VERIFY(home_altitude_.value() == home_altitude);
    BOOST_VERIFY(mission_altitude_ == mission_altitude);
  } else {
    home_altitude_ = home_altitude;
    mission_altitude_ = mission_altitude;
    spdlog::debug("home_altitude::set_altitude, home: {}, mission: {}", home_altitude_.value(), mission_altitude_);
  }

  const double diff{home_altitude_.value() + mission_altitude_ - drone_altitude};
  BOOST_VERIFY(std::abs(diff) < 1.0);
}

double home_altitude::get_home_altitude() const {
  BOOST_VERIFY(in_progress_);
  BOOST_VERIFY(home_altitude_.has_value());

  return home_altitude_.value();
}

void home_altitude::mission_stop() {
  BOOST_VERIFY(in_progress_);

  in_progress_ = false;
  home_altitude_.reset();
}