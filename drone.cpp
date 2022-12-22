#include "drone.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>

#include <future>

#include <dji_fc_subscription.h> // T_DjiFcSubscriptionQuaternion
#include <dji_mop_channel.h> // DjiMopChannel_Init

#include "api_code.h"
#include "server.h"

volatile sig_atomic_t drone::sigint_received_ = 0;

double drone::drone_yaw_{0.0};
double drone::drone_longitude_{0.0};
double drone::drone_latitude_{0.0};
int16_t drone::rc_mode_{-1};

mission_state drone::mission_state_;
std::mutex drone::m_;
std::list<interconnection::command_type::command_t> drone::execute_commands_;

T_DjiReturnCode drone::quaternion_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto quaternion{*(const T_DjiFcSubscriptionQuaternion*)data};
  (void)data_size;
  (void)timestamp;

  // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
  const double q2sqr{quaternion.q2 * quaternion.q2};
  const double t0{-2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0};
  const double t1{+2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3)};

  // https://sdk-forum.dji.net/hc/en-us/requests/74003
  // https://sdk-forum.dji.net/hc/en-us/articles/360023657273
  drone_yaw_ = atan2(t1, t0) * rad2deg; // Z

  BOOST_VERIFY(drone_yaw_ >= -180.0);
  BOOST_VERIFY(drone_yaw_ <= 180.0);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::rc_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto rc{*(const T_DjiFcSubscriptionRC*)data};
  (void)data_size;
  (void)timestamp;

  rc_mode_ = rc.mode;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::position_fused_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp) {
  BOOST_VERIFY(data != nullptr);
  const auto position{*(const T_DjiFcSubscriptionPositionFused*)data};
  (void)data_size;
  (void)timestamp;

  drone_latitude_ = position.latitude * rad2deg;
  drone_longitude_ = position.longitude * rad2deg;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::mission_event_callback(T_DjiWaypointV2MissionEventPush event_data) {
  if (!mission_state_.is_started()) {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  }

  mission_state_.update(event_data);

  if (!mission_state_.is_started()) {
    std::lock_guard<std::mutex> lock(m_);
    execute_commands_.push_back(
        interconnection::command_type::MISSION_FINISHED);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode drone::mission_state_callback(T_DjiWaypointV2MissionStatePush state_data) {
  if (!mission_state_.is_started()) {
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
  }

  mission_state_.update(state_data);

  if (!mission_state_.is_started()) {
    std::lock_guard<std::mutex> lock(m_);
    execute_commands_.push_back(
        interconnection::command_type::MISSION_FINISHED);
  }

  // https://developer.dji.com/onboard-api-reference/structDJI_1_1OSDK_1_1Telemetry_1_1RC.html#a9e69e1b32599986319ad3312ca5723de
  if (rc_mode_ != 8000) {
    // Value received while running tests on simulator
    spdlog::error("Unexpected RC mode: {}", rc_mode_);
    BOOST_VERIFY(false);
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

drone::drone() {
  T_DjiReturnCode code{DjiFcSubscription_Init()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
      quaternion_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RC, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
      rc_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
      position_fused_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_Init();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_RegisterMissionEventCallback(mission_event_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_RegisterMissionStateCallback(mission_state_callback);
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  interconnection::command_type command;
  command.set_type(interconnection::command_type::PING);
  command.set_version(protocol_version);
  std::string buffer_1;
  bool ok{command.SerializeToString(&buffer_1)};
  BOOST_VERIFY(ok);
  command_bytes_size_ = {static_cast<uint32_t>(buffer_1.size())};
  BOOST_VERIFY(command_bytes_size_ > 0);

  interconnection::pin_coordinates pin_coordinates;
  pin_coordinates.set_latitude(0.0);
  pin_coordinates.set_longitude(0.0);
  std::string buffer_2;
  ok = pin_coordinates.SerializeToString(&buffer_2);
  BOOST_VERIFY(ok);
  pin_coordinates_bytes_size_ = {static_cast<uint32_t>(buffer_2.size())};
  BOOST_VERIFY(pin_coordinates_bytes_size_ > 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void drone::sigint_handler(int signal) {
  (void)signal;
  sigint_received_ = 1;
}

void drone::check_sigint() {
  if (sigint_received_) {
    throw std::runtime_error("SIGINT received");
  }
}

drone::~drone() {
  T_DjiReturnCode code{DjiFcSubscription_DeInit()};
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  code = DjiWaypointV2_Deinit();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void drone::start() {
  spdlog::info("Protocol version: {}", protocol_version);
  spdlog::info("Command bytes size: {}", command_bytes_size_);

  T_DjiReturnCode code = DjiMopChannel_Init();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  while (true) {
    check_sigint();

    BOOST_VERIFY(signal(SIGINT, SIG_DFL) != SIG_ERR);
    server server{channel_id};
    channel_handle_ = server.handle();
    BOOST_VERIFY(channel_handle_ != nullptr);
    BOOST_VERIFY(signal(SIGINT, sigint_handler) != SIG_ERR);

    connection_closed_ = false;

    std::future<void> receive_data_future{
        std::async(std::launch::async, &drone::receive_data_job, this)};
    std::future<void> send_data_future{
        std::async(std::launch::async, &drone::send_data_job, this)};

    try {
      receive_data_future.get();
    } catch (pipeline_closed&) {
      connection_closed_ = true;
    } catch (std::exception& e) {
      connection_closed_ = true;
      throw e;
    }

    try {
      send_data_future.get();
    } catch (pipeline_closed&) {
      connection_closed_ = true;
    } catch (std::exception& e) {
      connection_closed_ = true;
      throw e;
    }
  }
}

void drone::receive_data_job() {
  spdlog::info("Received data job started");

  std::string buffer;
  buffer.resize(command_bytes_size_);

  while (true) {
    BOOST_VERIFY(command_bytes_size_ == buffer.size());
    receive_data(&buffer);

    interconnection::command_type command;
    const bool ok{command.ParseFromString(buffer)};
    BOOST_VERIFY(ok);

    switch (command.type()) {
      case interconnection::command_type::PING: {
        std::lock_guard<std::mutex> lock(m_);
        execute_commands_.push_back(command.type());
      } break;
      case interconnection::command_type::MISSION_START: {
        std::string buffer;
        buffer.resize(pin_coordinates_bytes_size_);
        receive_data(&buffer);

        if (mission_state_.is_started()) {
          spdlog::info("Mission resume");
          T_DjiReturnCode code{DjiWaypointV2_Resume()};
          BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
          // FIXME (verify mission state)
          continue;
        }

        interconnection::pin_coordinates pin_coordinates;
        const bool ok{pin_coordinates.ParseFromString(buffer)};
        BOOST_VERIFY(ok);

        const double lat{pin_coordinates.latitude()};
        const double lon{pin_coordinates.longitude()};

        srand(time(nullptr));

        T_DjiWayPointV2MissionSettings s;
        s.missionID = rand();  // Just a random number
        s.repeatTimes = 0;     // execute just once and go home
        s.finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
        s.maxFlightSpeed = 10;
        s.autoFlightSpeed = 2;
        s.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
        s.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;

        // FIXME (points from polygons)
        // FIXME (action at waypoint)
        waypoints_.clear();
        waypoints_.push_back(make_waypoint(lat, lon, 15.0F));
        waypoints_.push_back(make_waypoint(lat, lon, 20.0F));
        s.mission = waypoints_.data();

        s.missTotalLen = waypoints_.size();
        BOOST_VERIFY(s.missTotalLen >= 2);
        BOOST_VERIFY(s.missTotalLen <= 65535);

        spdlog::info("Mission start: lat({}), lon({}) (mission ID: {})", lat,
                     lon, s.missionID);

        T_DjiReturnCode code = DjiWaypointV2_UploadMission(&s);
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        code = DjiWaypointV2_Start();
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        mission_state_.start();
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_PAUSE: {
        spdlog::info("Mission pause");
        T_DjiReturnCode code{DjiWaypointV2_Pause()};
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_ABORT: {
        spdlog::info("Mission ABORT");
        // Call it first to block the update callbacks
        mission_state_.finish();
        T_DjiReturnCode code{DjiWaypointV2_Stop()};
        BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        // FIXME (verify mission state)
      } break;
      default:
        spdlog::error("Unexpected command type: {}", command.type());
        BOOST_VERIFY(false);
    }
  }
}

void drone::send_data_job() {
  spdlog::info("Send data job started");

  std::optional<interconnection::command_type::command_t> command;

  while (true) {
    command.reset();

    {
      std::lock_guard<std::mutex> lock(m_);
      if (!execute_commands_.empty()) {
        command = execute_commands_.front();
      }
    }

    if (!command.has_value()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      std::lock_guard<std::mutex> lock(m_);
      execute_commands_.push_back(
          interconnection::command_type::DRONE_COORDINATES);
      continue;
    }

    switch (command.value()) {
      case interconnection::command_type::PING: {
        spdlog::info("Execute PING command");
        send_command(interconnection::command_type::PING);
        break;
      }
      case interconnection::command_type::MISSION_FINISHED: {
        spdlog::info("Execute MISSION_FINISHED command");
        send_command(interconnection::command_type::MISSION_FINISHED);
      } break;
      case interconnection::command_type::DRONE_COORDINATES: {
        send_command(interconnection::command_type::DRONE_COORDINATES);

        interconnection::drone_coordinates dc;
        dc.set_latitude(drone_latitude_);
        dc.set_longitude(drone_longitude_);
        dc.set_heading(drone_yaw_);
        std::string buffer;
        const bool ok{dc.SerializeToString(&buffer)};
        BOOST_VERIFY(ok);

        send_data(buffer);

        spdlog::info("Drone coordinates sent: lat:{}, lon:{}, head:{}",
                     drone_latitude_, drone_longitude_, drone_yaw_);
        break;
      }
      default:
        spdlog::error("Unexpected command: {}", command.value());
        BOOST_VERIFY(false);
    }

    {
      std::lock_guard<std::mutex> lock(m_);
      execute_commands_.pop_front();
    }
  }
}

void drone::send_command(
    interconnection::command_type::command_t command_type) {
  interconnection::command_type command;
  command.set_type(command_type);
  command.set_version(protocol_version);
  std::string buffer;
  const bool ok{command.SerializeToString(&buffer)};
  BOOST_VERIFY(ok);
  BOOST_VERIFY(buffer.size() == command_bytes_size_);

  send_data(buffer);
}

void drone::receive_data(std::string* buffer) {
  BOOST_VERIFY(buffer != nullptr);
  BOOST_VERIFY(buffer->size() > 0);

  char* char_buffer{buffer->data()};
  static_assert(sizeof(char) == sizeof(uint8_t));
  uint8_t* recv_buf{reinterpret_cast<uint8_t*>(char_buffer)};

  while (true) {
    if (connection_closed_) {
      throw pipeline_closed();
    }
    check_sigint();

    uint32_t real_len{0};
    BOOST_VERIFY(channel_handle_ != nullptr);
    const api_code code{DjiMopChannel_RecvData(channel_handle_, recv_buf, buffer->size(), &real_len)};
    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(real_len == buffer->size());
    spdlog::info("{} bytes received", real_len);
    return;
  }
}

void drone::send_data(std::string& buffer) {
  BOOST_VERIFY(buffer.size() > 0);

  char* char_buffer{buffer.data()};
  static_assert(sizeof(char) == sizeof(uint8_t));
  uint8_t* send_buf{reinterpret_cast<uint8_t*>(char_buffer)};

  while (true) {
    if (connection_closed_) {
      throw pipeline_closed();
    }
    check_sigint();

    uint32_t real_len{0};
    BOOST_VERIFY(channel_handle_ != nullptr);
    const api_code code{DjiMopChannel_SendData(channel_handle_, send_buf, buffer.size(), &real_len)};
    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(real_len == buffer.size());
    spdlog::info("{} bytes sent", real_len);
    return;
  }
}

T_DjiWaypointV2 drone::make_waypoint(double latitude, double longitude,
                                           float relative_height) {
  T_DjiWaypointV2 p;

  p.latitude = latitude * deg2rad;
  p.longitude = longitude * deg2rad;
  p.relativeHeight = relative_height;

  p.waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
  p.headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;

  p.config.useLocalCruiseVel = 0;  // set local waypoint's cruise speed
  p.config.useLocalMaxVel = 0;     // set local waypoint's max speed

  p.dampingDistance = 40;  // cm
  p.heading = 0.0;         // unused?

  p.turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

  // unused
  p.pointOfInterest.positionX = 0.0F;
  p.pointOfInterest.positionY = 0.0F;
  p.pointOfInterest.positionZ = 0.0F;

  p.maxFlightSpeed = 10.0F;
  p.autoFlightSpeed = 2.0F;

  return p;
}
