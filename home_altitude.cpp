#include "home_altitude.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY
#include <numeric>           // std::accumulate

home_altitude::home_altitude() = default;

void home_altitude::mission_start() {
  BOOST_VERIFY(!in_progress_);
  BOOST_VERIFY(!home_altitude_.has_value());

  in_progress_ = true;
}

void home_altitude::set_altitude(float drone_altitude, float mission_altitude,
                                 float home_altitude) {
  BOOST_VERIFY(in_progress_);

  if (home_altitude_.has_value()) {
    BOOST_VERIFY(home_altitude_.value() == home_altitude);
  } else {
    home_altitude_ = home_altitude;
  }

  const float diff{home_altitude_.value() + mission_altitude - drone_altitude};
  BOOST_VERIFY(std::abs(diff) < 1.0F);
}

auto home_altitude::get_home_altitude() const -> float {
  BOOST_VERIFY(in_progress_);
  BOOST_VERIFY(home_altitude_.has_value());

  // https://github.com/llvm/llvm-project/issues/60661
  return home_altitude_.value();  // NOLINT (bugprone-unchecked-optional-access)
}

void home_altitude::mission_stop() {
  BOOST_VERIFY(in_progress_);

  in_progress_ = false;
  home_altitude_.reset();
}
