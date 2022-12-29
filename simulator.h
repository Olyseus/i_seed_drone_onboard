#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <cmath>  // M_PI
#include <string>

#include "api_code.h"

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
class simulator {
 public:
  simulator();
  ~simulator();

  simulator(const simulator&) = delete;
  simulator(simulator&&) = delete;

  simulator& operator=(const simulator&) = delete;
  simulator& operator=(simulator&&) = delete;

  void gps_callback(double latitude, double longitude);

  api_code receive_data(std::string* buffer);

 private:
  void verify_lat_lon();

  enum state {
    begin,
    mission_start,
    end
  };

  state state_{begin};

  double latitude_{0.0};
  double longitude_{0.0};

  static constexpr double p1_lat_{48.85};
  static constexpr double p1_lon_{2.40};

  static constexpr double p2_lat_{48.85};
  static constexpr double p2_lon_{2.4003};
};
#endif

#endif // SIMULATOR_H_
