#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <cmath>  // M_PI
#include <mutex>
#include <optional>
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

  // thread: callback
  void gps_callback(double latitude, double longitude);

  // thread: receive_data
  api_code receive_data(std::string* buffer);

  // thread: action
  void laser_range(double range);

 private:
  std::mutex m_;

  void verify_lat_lon();

  enum state {
    begin,
    mission_start,
    end
  };

  state state_{begin};

  double latitude_{0.0};
  double longitude_{0.0};

  static constexpr double p1_lat_{48.90};
  static constexpr double p1_lon_{-9.40};

  static constexpr double p2_lat_{48.90};
  static constexpr double p2_lon_{-9.4003};

  std::optional<double> laser_range_;
  bool laser_range_cmd_sent_{false};
};
#endif

#endif // SIMULATOR_H_
