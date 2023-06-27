#include "simulator.h"

#include <spdlog/spdlog.h>

#include <thread>  // std::this_thread

#include "drone.h"
#include "olyseus_verify.h"  // OLYSEUS_VERIFY

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR) && \
    !defined(I_SEED_DRONE_ONBOARD_INTERCONNECTION)

simulator::simulator() {
  // RC is off, return values according to simulated mission

  // waypoint 0, bad
  laser_range_values_.push_back(13.2F);
  laser_range_values_.push_back(15.0F);

  // waypoint 1, good
  laser_range_values_.push_back(15.2F);

  // waypoint 2, good
  laser_range_values_.push_back(14.9F);

  // waypoint 3, bad
  laser_range_values_.push_back(17.8F);
  laser_range_values_.push_back(15.0F);

  // fake waypoint
  laser_range_values_.push_back(15.0F);

  // backward

  // waypoint 3
  laser_range_values_.push_back(15.2F);

  // waypoint 2
  laser_range_values_.push_back(14.8F);

  // waypoint 1
  laser_range_values_.push_back(15.1F);

  // waypoint 0
  laser_range_values_.push_back(15.5F);

  // extra data for laser measurement
  for (int i{0}; i < 10; ++i) {
    laser_range_values_.push_back(15.0F);
  }
}

simulator::~simulator() = default;

api_code simulator::receive_data(std::string* buffer) {
  OLYSEUS_VERIFY(buffer != nullptr);
  OLYSEUS_VERIFY(buffer->size() > 0);

  verify_lat_lon();

  const double p1_diff{std::abs(latitude_ - p1_lat_) +
                       std::abs(longitude_ - p1_lon_)};
  const double p2_diff{std::abs(latitude_ - p2_lat_) +
                       std::abs(longitude_ - p2_lon_)};

  const double mission_lat{(p1_diff > p2_diff) ? p1_lat_ : p2_lat_};
  const double mission_lon{(p1_diff > p2_diff) ? p1_lon_ : p2_lon_};

  interconnection::command_type command;
  command.set_type(interconnection::command_type::MISSION_START);
  command.set_version(drone::protocol_version);

  constexpr int32_t event_id{1};
  interconnection::pin_coordinates pin_coordinates;
  pin_coordinates.set_latitude(mission_lat);
  pin_coordinates.set_longitude(mission_lon);
  pin_coordinates.set_event_id(event_id);

  switch (state_) {
    case begin_size: {
      spdlog::info("simulate send: begin_size");
      std::string tmp_buffer;
      bool ok{command.SerializeToString(&tmp_buffer)};
      OLYSEUS_VERIFY(ok);

      interconnection::packet_size p_size;
      p_size.set_size(tmp_buffer.size());
      ok = p_size.SerializeToString(buffer);
      OLYSEUS_VERIFY(ok);
      state_ = begin;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case begin: {
      spdlog::info("simulate send: begin");
      const bool ok{command.SerializeToString(buffer)};
      OLYSEUS_VERIFY(ok);
      state_ = mission_start_size;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case mission_start_size: {
      spdlog::info("simulate send: mission_start_size");
      std::string tmp_buffer;
      bool ok{pin_coordinates.SerializeToString(&tmp_buffer)};
      OLYSEUS_VERIFY(ok);

      interconnection::packet_size p_size;
      p_size.set_size(tmp_buffer.size());
      ok = p_size.SerializeToString(buffer);
      OLYSEUS_VERIFY(ok);
      state_ = mission_start;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case mission_start: {
      spdlog::info("simulate send: mission_start");
      const bool ok{pin_coordinates.SerializeToString(buffer)};
      OLYSEUS_VERIFY(ok);
      state_ = end;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case end:
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT};
    default:
      OLYSEUS_UNREACHABLE;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER};
  }
}

auto simulator::laser_range() -> float {
  OLYSEUS_VERIFY(!laser_range_values_.empty());
  const float result{laser_range_values_.front()};
  laser_range_values_.pop_front();
  spdlog::info("Simulating laser range: {}", result);
  return result;
}

void simulator::verify_lat_lon() {
  OLYSEUS_VERIFY(std::abs(latitude_) > 1e-2);
  OLYSEUS_VERIFY(std::abs(longitude_) > 1e-2);

  OLYSEUS_VERIFY(std::abs(latitude_ - p1_lat_) < 0.01);
  OLYSEUS_VERIFY(std::abs(longitude_ - p1_lon_) < 0.01);
}

#endif
