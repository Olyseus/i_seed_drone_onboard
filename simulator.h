#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <atomic>
#include <cmath>  // M_PI
#include <list>
#include <mutex>
#include <optional>
#include <string>

#include "api_code.h"

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
#if defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)
class simulator {
 public:
  simulator() = default;
  ~simulator() = default;

  simulator(const simulator&) = delete;
  simulator(simulator&&) = delete;

  simulator& operator=(const simulator&) = delete;
  simulator& operator=(simulator&&) = delete;

  // thread: action
  float laser_range() { return 15.0F; }
};
#else
class simulator {
 public:
  simulator();
  ~simulator();

  simulator(const simulator&) = delete;
  simulator(simulator&&) = delete;

  simulator& operator=(const simulator&) = delete;
  simulator& operator=(simulator&&) = delete;

  // Callback thread, do not block execution
  void gps_callback(double latitude, double longitude) {
    latitude_ = latitude;
    longitude_ = longitude;
  }

  // thread: receive_data
  api_code receive_data(std::string* buffer);

  // thread: action
  float laser_range();

 private:
  void verify_lat_lon();

  enum state {
    build_mission_size,
    build_mission,
    input_polygon_size,
    input_polygon_packet,
    mission_start_size,
    mission_start,
    event_id_message_size,
    event_id_message_packet,
    end
  };
  state state_{build_mission_size};

  std::atomic<double> latitude_{0.0};
  std::atomic<double> longitude_{0.0};

  static constexpr double p1_lat_{48.90};
  static constexpr double p1_lon_{-9.40};

  static constexpr double p2_lat_{48.90};
  static constexpr double p2_lon_{-9.4003};

  std::list<float> laser_range_values_;
};
#endif
#endif

#endif  // SIMULATOR_H_
