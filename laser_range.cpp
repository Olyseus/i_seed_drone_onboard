#include "laser_range.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
#include "simulator.h"
#endif

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
laser_range::laser_range(simulator& s) : simulator_(s) {
  // waypoint 0, bad
  values_.push_back(13.2);
  values_.push_back(15.0);

  // waypoint 1, good
  values_.push_back(15.2);

  // waypoint 2, good
  values_.push_back(14.9);

  // waypoint 3, bad
  values_.push_back(17.8);
  values_.push_back(15.0);

  // fake waypoint
  values_.push_back(15.0);

  // backward

  // waypoint 3
  values_.push_back(15.2);

  // waypoint 2
  values_.push_back(14.8);

  // waypoint 1
  values_.push_back(15.1);

  // waypoint 0
  values_.push_back(15.5);

  // extra data for laser measurement
  for (int i{0}; i < 10; ++i) {
    values_.push_back(15.0);
  }
}
#else
laser_range::laser_range() = default;
#endif

laser_range::~laser_range() = default;

// thread: receive_data
void laser_range::value_received(double range) {
  {
    const std::lock_guard<std::mutex> lock(m_);

    latest_time_point_ = clock::now();
    BOOST_VERIFY(latest_time_point_ != invalid_time_point);

    latest_value_ = range;
    spdlog::info("Received laser range: {}", latest_value_);
  }
  condition_variable_.notify_one();
}

// thread: action
auto laser_range::latest(
    std::mutex& m,
    std::list<interconnection::command_type::command_t>& commands) -> double {
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  {
    const std::lock_guard<std::mutex> lock(m_);
    BOOST_VERIFY(!values_.empty());
    const double simulated_range{values_.front()};
    values_.pop_front();
    spdlog::info("Simulating laser range: {}", simulated_range);
    simulator_.laser_range(simulated_range);
  }
#endif

  while (true) {
    {
      const std::lock_guard<std::mutex> lock(m_);
      if (latest_time_point_ == invalid_time_point) {
        spdlog::info("No laser data available");
      } else {
        constexpr int64_t window_ms{2000};
        const auto elapsed{clock::now() - latest_time_point_};
        namespace ch = std::chrono;
        if (ch::duration_cast<ch::milliseconds>(elapsed).count() > window_ms) {
          spdlog::info("Laser data is outdated");
          latest_time_point_ = invalid_time_point;
        } else {
          return latest_value_;
        }
      }

      BOOST_VERIFY(latest_time_point_ == invalid_time_point);
    }

    {
      std::lock_guard<std::mutex> lock(m);
      commands.push_back(interconnection::command_type::LASER_RANGE);
    }

    // need to wait for a new value
    std::unique_lock lock{m_};
    condition_variable_.wait(
        lock, [this] { return latest_time_point_ != invalid_time_point; });
    BOOST_VERIFY(latest_time_point_ != invalid_time_point);
  }
}
