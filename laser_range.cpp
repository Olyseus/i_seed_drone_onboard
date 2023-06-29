#include "laser_range.h"

#include <spdlog/spdlog.h>

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

#if !defined(I_SEED_DRONE_ONBOARD_SIMULATOR)

laser_range::laser_range() = default;
laser_range::~laser_range() = default;

// thread: receive_data
void laser_range::value_received(float range) {
  {
    const std::lock_guard lock{m_};

    latest_time_point_ = clock::now();
    OLYSEUS_VERIFY(latest_time_point_ != invalid_time_point);

    latest_value_ = range;
    spdlog::info("Received laser range: {}", latest_value_);
  }
  condition_variable_.notify_one();
}

// thread: action
auto laser_range::latest(
    std::mutex& m,
    std::list<interconnection::command_type::command_t>& commands) -> float {
  while (true) {
    {
      const std::lock_guard lock{m_};
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

      OLYSEUS_VERIFY(latest_time_point_ == invalid_time_point);
    }

    {
      const std::lock_guard lock{m};
      commands.push_back(interconnection::command_type::LASER_RANGE_REQUEST);
    }

    // need to wait for a new value
    std::unique_lock lock{m_};
    condition_variable_.wait(
        lock, [this] { return latest_time_point_ != invalid_time_point; });
    OLYSEUS_VERIFY(latest_time_point_ != invalid_time_point);
  }
}

#endif
