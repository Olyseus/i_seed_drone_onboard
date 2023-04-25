#include "home_altitude.h"

#include <spdlog/spdlog.h>

#include <numeric>  // std::accumulate

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

home_altitude::home_altitude() = default;

void home_altitude::mission_start() {
  OLYSEUS_VERIFY(!in_progress_);
  OLYSEUS_VERIFY(!home_altitude_.has_value());

  in_progress_ = true;
}

void home_altitude::set_altitude(float drone_altitude, float mission_altitude,
                                 float home_altitude) {
  OLYSEUS_VERIFY(in_progress_);

  if (home_altitude_.has_value()) {
    OLYSEUS_VERIFY(home_altitude_.value() == home_altitude);
  } else {
    home_altitude_ = home_altitude;
  }

  const float diff{home_altitude_.value() + mission_altitude - drone_altitude};
  OLYSEUS_VERIFY(std::abs(diff) < 1.0F);
}

auto home_altitude::get_home_altitude() const -> float {
  OLYSEUS_VERIFY(in_progress_);
  OLYSEUS_VERIFY(home_altitude_.has_value());

  // https://github.com/llvm/llvm-project/issues/60661
  return home_altitude_.value();  // NOLINT (bugprone-unchecked-optional-access)
}

void home_altitude::mission_stop() {
  OLYSEUS_VERIFY(in_progress_);

  in_progress_ = false;
  home_altitude_.reset();
}
