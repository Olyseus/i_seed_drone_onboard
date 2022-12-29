#include "simulator.h"

#include <boost/assert.hpp> // BOOST_VERIFY
#include <thread> // std::this_thread

#include "drone.h"

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)

simulator::simulator() = default;
simulator::~simulator() = default;

void simulator::gps_callback(double latitude, double longitude) {
  latitude_ = latitude;
  longitude_ = longitude;

  verify_lat_lon();
}

api_code simulator::receive_data(std::string* buffer) {
  BOOST_VERIFY(buffer != nullptr);
  BOOST_VERIFY(buffer->size() > 0);

  verify_lat_lon();

  const double p1_diff{std::abs(latitude_ - p1_lat_) + std::abs(longitude_ - p1_lon_)};
  const double p2_diff{std::abs(latitude_ - p2_lat_) + std::abs(longitude_ - p2_lon_)};

  const double mission_lat{(p1_diff > p2_diff) ? p1_lat_ : p2_lat_};
  const double mission_lon{(p1_diff > p2_diff) ? p1_lon_ : p2_lon_};

  switch (state_) {
    case begin: {
      interconnection::command_type command;
      command.set_type(interconnection::command_type::MISSION_START);
      command.set_version(drone::protocol_version);
      const bool ok{command.SerializeToString(buffer)};
      BOOST_VERIFY(ok);
      state_ = mission_start;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case mission_start: {
      interconnection::pin_coordinates pin_coordinates;
      pin_coordinates.set_latitude(mission_lat);
      pin_coordinates.set_longitude(mission_lon);
      const bool ok{pin_coordinates.SerializeToString(buffer)};
      BOOST_VERIFY(ok);
      state_ = end;
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS};
    }
    case end:
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT};
    default:
      BOOST_VERIFY(false);
      return api_code{DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER};
  }
}

void simulator::verify_lat_lon() {
  BOOST_VERIFY(std::abs(latitude_) > 1e-2);
  BOOST_VERIFY(std::abs(longitude_) > 1e-2);

  BOOST_VERIFY(std::abs(latitude_ - p1_lat_) < 0.01);
  BOOST_VERIFY(std::abs(longitude_ - p1_lon_) < 0.01);
}

#endif