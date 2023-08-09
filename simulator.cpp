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

  // waypoint 2, bad
  laser_range_values_.push_back(17.8F);
  laser_range_values_.push_back(15.0F);

  // fake waypoint
  laser_range_values_.push_back(15.0F);

  // backward

  // waypoint 2
  laser_range_values_.push_back(15.2F);

  // waypoint 1
  laser_range_values_.push_back(14.8F);

  // waypoint 0
  laser_range_values_.push_back(15.5F);

  // extra data for laser measurement
  for (int i{0}; i < 1000; ++i) {
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

  interconnection::command_type build_mission_cmd;
  build_mission_cmd.set_type(interconnection::command_type::BUILD_MISSION);
  build_mission_cmd.set_version(drone::protocol_version);
  constexpr int32_t build_mission_event_id{1};

  interconnection::input_polygon input_polygon;

  constexpr double side{0.00009};

  interconnection::coordinate* v_1{input_polygon.add_vertices()};
  v_1->set_latitude(mission_lat - side);
  v_1->set_longitude(mission_lon - side);

  interconnection::coordinate* v_2{input_polygon.add_vertices()};
  v_2->set_latitude(mission_lat - side);
  v_2->set_longitude(mission_lon + side);

  interconnection::coordinate* v_3{input_polygon.add_vertices()};
  v_3->set_latitude(mission_lat + side);
  v_3->set_longitude(mission_lon + side);

  interconnection::coordinate* v_4{input_polygon.add_vertices()};
  v_4->set_latitude(mission_lat + side);
  v_4->set_longitude(mission_lon - side);

  input_polygon.set_event_id(build_mission_event_id);

  input_polygon.mutable_home()->set_latitude(mission_lat);
  input_polygon.mutable_home()->set_longitude(mission_lon);

  interconnection::command_type mission_start_cmd;
  mission_start_cmd.set_type(interconnection::command_type::MISSION_START);
  mission_start_cmd.set_version(drone::protocol_version);
  constexpr int32_t mission_start_event_id{2};

  interconnection::event_id_message event_id_message;
  event_id_message.set_event_id(mission_start_event_id);

  switch (state_) {
    case build_mission_size: {
      spdlog::info("simulate send: build_mission_size");

      std::string tmp_buffer;
      bool ok{build_mission_cmd.SerializeToString(&tmp_buffer)};
      OLYSEUS_VERIFY(ok);

      interconnection::packet_size p_size;
      p_size.set_size(tmp_buffer.size());
      ok = p_size.SerializeToString(buffer);
      OLYSEUS_VERIFY(ok);

      state_ = build_mission;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case build_mission: {
      spdlog::info("simulate send: build_mission");

      const bool ok{build_mission_cmd.SerializeToString(buffer)};
      OLYSEUS_VERIFY(ok);

      state_ = input_polygon_size;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case input_polygon_size: {
      spdlog::info("simulate send: input_polygon_size");

      std::string tmp_buffer;
      bool ok{input_polygon.SerializeToString(&tmp_buffer)};
      OLYSEUS_VERIFY(ok);

      interconnection::packet_size p_size;
      p_size.set_size(tmp_buffer.size());
      ok = p_size.SerializeToString(buffer);
      OLYSEUS_VERIFY(ok);

      state_ = input_polygon_packet;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case input_polygon_packet: {
      spdlog::info("simulate send: input_polygon_packet");

      const bool ok{input_polygon.SerializeToString(buffer)};
      OLYSEUS_VERIFY(ok);

      state_ = mission_start_size;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case mission_start_size: {
      spdlog::info("simulate send: mission_start_size");

      std::string tmp_buffer;
      bool ok{mission_start_cmd.SerializeToString(&tmp_buffer)};
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

      const bool ok{mission_start_cmd.SerializeToString(buffer)};
      OLYSEUS_VERIFY(ok);

      state_ = event_id_message_size;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case event_id_message_size: {
      spdlog::info("simulate send: event_id_message_size");

      std::string tmp_buffer;
      bool ok{event_id_message.SerializeToString(&tmp_buffer)};
      OLYSEUS_VERIFY(ok);

      interconnection::packet_size p_size;
      p_size.set_size(tmp_buffer.size());
      ok = p_size.SerializeToString(buffer);
      OLYSEUS_VERIFY(ok);

      state_ = event_id_message_packet;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case event_id_message_packet: {
      spdlog::info("simulate send: event_id_message_packet");

      const bool ok{event_id_message.SerializeToString(buffer)};
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
