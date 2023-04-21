#ifndef LASER_RANGE_H_
#define LASER_RANGE_H_

#include <chrono>
#include <condition_variable>
#include <list>
#include <mutex>

#include "interconnection.pb.h"

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
class simulator;
#endif

/// \brief Information about the laser range measurement
class laser_range {
 public:
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  laser_range(simulator& s);
#else
  laser_range();
#endif

  ~laser_range();

  /// \cond private
  laser_range(const laser_range&) = delete;
  laser_range(laser_range&&) = delete;

  laser_range& operator=(const laser_range&) = delete;
  laser_range& operator=(laser_range&&) = delete;
  /// \endcond

  /// \brief Save the received laser range value
  /// \note Thread: receive_data
  void value_received(float range);

  /// \brief Request the laser range measurement
  /// \param[in] m Mutex for locking the commands list
  /// \param[in] commands List of commands where to put the laser request
  /// \note Thread: action
  float latest(std::mutex& m,
               std::list<interconnection::command_type::command_t>& commands);

 private:
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  simulator& simulator_;
  std::list<float> values_;
#endif
  std::mutex m_;
  std::condition_variable condition_variable_;

  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  static constexpr time_point invalid_time_point{time_point::min()};

  time_point latest_time_point_{invalid_time_point};
  float latest_value_{0.0};
};

#endif  // LASER_RANGE_H_
