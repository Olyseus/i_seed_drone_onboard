#include "drone.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>
#include <dji_linux_helpers.hpp>  // LinuxSetup
#include <future>

template <>
api_code::api_code(const ACK::ErrorCode& error_code) {
  if (ACK::getError(error_code) == ACK::SUCCESS) {
    code_ = code::success;
    return;
  }
  BOOST_VERIFY(ACK::getError(error_code) == ACK::FAIL);
  ACK::getErrorCodeMessage(error_code, __func__);
}

template <>
api_code::api_code(const ErrorCode::ErrorCodeType& error_code) {
  if (error_code == ErrorCode::SysCommonErr::Success) {
    code_ = code::success;
    return;
  }
  ErrorCode::printErrorCodeMsg(error_code);
}

// https://github.com/dji-sdk/Onboard-SDK/blob/4.1.0/osdk-core/modules/inc/mop/dji_mop_define.hpp#L49
template <>
api_code::api_code(const DJI::OSDK::MOP::MopErrCode& error_code) {
  switch (error_code) {
    case MOP_PASSED:
      code_ = code::success;
      return;
    case MOP_FAILED:
      spdlog::error("MOP_FAILED");
      return;
    case MOP_CRC:
      spdlog::error("MOP_CRC");
      return;
    case MOP_PARM:
      spdlog::error("MOP_PARM");
      return;
    case MOP_NOMEM:
      spdlog::error("MOP_NOMEM");
      throw pipeline_closed();
    case MOP_NOTREADY:
      spdlog::error("MOP_NOTREADY");
      break; // retry
    case MOP_SEND:
      spdlog::error("MOP_SEND");
      return;
    case MOP_RECV:
      spdlog::error("MOP_RECV");
      return;
    case MOP_TIMEOUT:
      spdlog::error("MOP_TIMEOUT");
      break; // retry
    case MOP_RESBUSY:
      spdlog::error("MOP_RESBUSY");
      break; // retry
    case MOP_RESOCCUPIED:
      spdlog::error("MOP_RESOCCUPIED");
      break; // retry
    case MOP_CONNECTIONCLOSE:
      spdlog::error("MOP_CONNECTIONCLOSE");
      throw pipeline_closed();
    case MOP_NOTIMPLEMENT:
      spdlog::error("MOP_NOTIMPLEMENT");
      return;
    case MOP_UNKNOWN_ERR:
      spdlog::error("MOP_UNKNOWN_ERR");
      return;
    default:
      spdlog::error("Invalid MopErrCode: {}", error_code);
      return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  code_ = code::retry;
}

drone::drone(int argc, char** argv) {
  spdlog::info("Setup for Linux");
  constexpr bool enable_advanced_sensing{true};
  linux_setup_.reset(new LinuxSetup(argc, argv, enable_advanced_sensing));
  vehicle_ = linux_setup_->getVehicle();
  BOOST_VERIFY(vehicle_);

  constexpr int freq{5};
  DJI::OSDK::Telemetry::TopicName topic_list[] = {
      DJI::OSDK::Telemetry::TOPIC_QUATERNION,
      DJI::OSDK::Telemetry::TOPIC_GPS_FUSED};
  constexpr std::size_t num_topic{sizeof(topic_list) / sizeof(topic_list[0])};
  constexpr bool enable_timestamp{false};

  const bool pkg_status = vehicle_->subscribe->initPackageFromTopicList(
      pkg_index, num_topic, topic_list, enable_timestamp, freq);
  BOOST_VERIFY(pkg_status);

  const api_code code{vehicle_->subscribe->startPackage(pkg_index, timeout)};
  BOOST_VERIFY(code.success());

  spdlog::info("Setup finished");

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

drone::~drone() {
  if (vehicle_) {
    vehicle_->subscribe->removePackage(pkg_index, timeout);
  }
}

void drone::start() {
  spdlog::info("Protocol version: {}", protocol_version);
  spdlog::info("Command bytes size: {}", command_bytes_size_);

  while (true) {
    std::unique_ptr<MopServer> server{new MopServer()};

    spdlog::info("Creating channel {}", channel_id);
    api_code code{server->accept(channel_id, MOP::PipelineType::RELIABLE, pipeline_)};
    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(pipeline_ != nullptr);

    connection_closed_ = false;

    std::future<void> receive_data_future{
        std::async(std::launch::async, &drone::receive_data_job, this)};
    std::future<void> send_data_future{
        std::async(std::launch::async, &drone::send_data_job, this)};

    try {
      receive_data_future.get();
    } catch (pipeline_closed&) {
      connection_closed_ = true;
    }
    try {
      send_data_future.get();
    } catch (pipeline_closed&) {
      connection_closed_ = true;
    }

    code = api_code{server->close(channel_id)};
    BOOST_VERIFY(code.success());
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

        if (mission_is_started_) {
          spdlog::info("Mission resume");
          const api_code code{vehicle_->waypointV2Mission->resume(timeout)};
          BOOST_VERIFY(code.success());
          // FIXME (verify mission state)
          continue;
        }

        interconnection::pin_coordinates pin_coordinates;
        const bool ok{pin_coordinates.ParseFromString(buffer)};
        BOOST_VERIFY(ok);

        const double lat{pin_coordinates.latitude()};
        const double lon{pin_coordinates.longitude()};

        spdlog::info("Mission start: lat({}), lon({})", lat, lon);

        api_code code{vehicle_->control->obtainCtrlAuthority(timeout)};
        BOOST_VERIFY(code.success());

        WayPointV2InitSettings s;
        s.missionID = 2573;  // Just a random number
        s.repeatTimes = 0;   // execute just once and go home
        s.finishedAction = DJIWaypointV2MissionFinishedGoHome;
        s.maxFlightSpeed = 10;
        s.autoFlightSpeed = 2;
        s.exitMissionOnRCSignalLost =
            0;  // continue mission even if signal is lost
        s.gotoFirstWaypointMode =
            DJIWaypointV2MissionGotoFirstWaypointModeSafely;

        // FIXME (points from polygons)
        // FIXME (action at waypoint)
        s.mission.clear();
        s.mission.push_back(make_waypoint(lat, lon, 15.0F));
        s.mission.push_back(make_waypoint(lat, lon, 20.0F));

        s.missTotalLen = s.mission.size();
        BOOST_VERIFY(s.missTotalLen >= 2);
        BOOST_VERIFY(s.missTotalLen <= 65535);

        code = api_code{vehicle_->waypointV2Mission->init(&s, timeout)};
        BOOST_VERIFY(code.success());

        code = api_code{vehicle_->waypointV2Mission->uploadMission(timeout)};
        BOOST_VERIFY(code.success());

        code = api_code{vehicle_->waypointV2Mission->start(timeout)};
        BOOST_VERIFY(code.success());

        vehicle_->waypointV2Mission->RegisterMissionStateCallback(
            this, update_mission_state);

        mission_is_started_ = true;
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_PAUSE: {
        spdlog::info("Mission pause");
        const api_code code{vehicle_->waypointV2Mission->pause(timeout)};
        BOOST_VERIFY(code.success());
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_ABORT: {
        spdlog::info("Mission abort");
        const api_code code{vehicle_->waypointV2Mission->stop(timeout)};
        BOOST_VERIFY(code.success());
        // FIXME (verify mission state)
        mission_finished();
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

        DJI::OSDK::Telemetry::Quaternion quaternion{
            vehicle_->subscribe
                ->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>()};
        DJI::OSDK::Telemetry::GPSFused global{
            vehicle_->subscribe
                ->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>()};

        const double latitude{global.latitude * 180.0 / M_PI};
        const double longitude{global.longitude * 180.0 / M_PI};

        // https://github.com/dji-sdk/Onboard-SDK/blob/2c38de17f7aad0064056f27eaa219d4ed30ab82a/sample/platform/STM32/OnBoardSDK_STM32/User/FlightControlSample.cpp#L800-L824
        const double q2sqr{quaternion.q2 * quaternion.q2};
        const double t0{-2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0};
        const double t1{+2.0 * (quaternion.q1 * quaternion.q2 +
                                quaternion.q0 * quaternion.q3)};
        const double heading{atan2(t1, t0) * 180.0 / M_PI};

        interconnection::drone_coordinates dc;
        dc.set_latitude(latitude);
        dc.set_longitude(longitude);
        dc.set_heading(heading);
        std::string buffer;
        const bool ok{dc.SerializeToString(&buffer)};
        BOOST_VERIFY(ok);

        send_data(buffer);

        spdlog::info("Drone coordinates sent: lat:{}, lon:{}, head:{}",
                     latitude, longitude, heading);
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
  MopPipeline::DataPackType recv_pack = {recv_buf,
                                         static_cast<uint32_t>(buffer->size())};
  uint32_t len{0};

  while (true) {
    if (connection_closed_) {
      throw pipeline_closed();
    }
    const api_code code{pipeline_->recvData(recv_pack, &len)};
    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(len == buffer->size());
    spdlog::info("{} bytes received", len);
    return;
  }
}

void drone::send_data(std::string& buffer) {
  BOOST_VERIFY(buffer.size() > 0);

  char* char_buffer{buffer.data()};
  static_assert(sizeof(char) == sizeof(uint8_t));
  uint8_t* send_buf{reinterpret_cast<uint8_t*>(char_buffer)};
  MopPipeline::DataPackType send_pack = {send_buf,
                                        static_cast<uint32_t>(buffer.size())};
  uint32_t len{0};

  while (true) {
    if (connection_closed_) {
      throw pipeline_closed();
    }
    const api_code code{pipeline_->sendData(send_pack, &len)};
    if (code.retry()) {
      continue;
    }
    BOOST_VERIFY(code.success());
    BOOST_VERIFY(len == buffer.size());
    spdlog::info("{} bytes sent", len);
    return;
  }
}

DJI::OSDK::WaypointV2 drone::make_waypoint(double latitude, double longitude,
                                           float relative_height) {
  WaypointV2 p;

  p.latitude = latitude * M_PI / 180.0;
  p.longitude = longitude * M_PI / 180.0;
  p.relativeHeight = relative_height;

  p.waypointType = DJIWaypointV2FlightPathModeGoToPointInAStraightLineAndStop;
  p.headingMode = DJIWaypointV2HeadingModeAuto;

  p.config.useLocalCruiseVel = 0;  // set local waypoint's cruise speed
  p.config.useLocalMaxVel = 0;     // set local waypoint's max speed

  p.dampingDistance = 40;  // cm
  p.heading = 0.0;         // unused?

  p.turnMode = DJIWaypointV2TurnModeClockwise;

  // unused
  p.pointOfInterest.positionX = 0.0F;
  p.pointOfInterest.positionY = 0.0F;
  p.pointOfInterest.positionZ = 0.0F;

  p.maxFlightSpeed = 10.0F;
  p.autoFlightSpeed = 2.0F;

  return p;
}

void drone::mission_finished() {
  BOOST_VERIFY(mission_is_started_);
  mission_is_started_ = false;
}

E_OsdkStat drone::update_mission_state(T_CmdHandle* cmd_handle,
                                       const T_CmdInfo* cmd_info,
                                       const uint8_t* cmd_data,
                                       void* user_data) {
  BOOST_VERIFY(user_data != nullptr);
  auto* self{static_cast<drone*>(user_data)};

  BOOST_VERIFY(cmd_data != nullptr);
  auto* ack{reinterpret_cast<const DJI::OSDK::MissionStatePushAck*>(cmd_data)};

  // Handle negative values correctly
  const int8_t state_i8{static_cast<int8_t>(ack->data.state)};
  const auto state{static_cast<DJI::OSDK::DJIWaypointV2MissionState>(state_i8)};

  const uint16_t waypoint_index{ack->data.curWaypointIndex};

  BOOST_VERIFY(cmd_handle != nullptr);
  BOOST_VERIFY(cmd_info != nullptr);

  if (!self->mission_is_started_) {
    return OSDK_STAT_OK;
  }

  bool finished{false};

  switch (state) {
    case DJIWaypointV2MissionStateUnWaypointActionActuatorknown:
      spdlog::info("Mission state unknown");
      break;
    case DJIWaypointV2MissionStateDisconnected:
      spdlog::info("Mission state disconnected");
      break;
    case DJIWaypointV2MissionStateReadyToExecute:
      spdlog::info("Mission state ready to execute");
      break;
    case DJIWaypointV2MissionStateExecuting:
      spdlog::info("Mission state executing, waypoint: {}", waypoint_index);
      break;
    case DJIWaypointV2MissionStateInterrupted:
      spdlog::info("Mission state interrupted, waypoint: {}", waypoint_index);
      break;
    case DJIWaypointV2MissionStateResumeAfterInterrupted:
      spdlog::info("Mission state resumed");
      break;
    case DJIWaypointV2MissionStateExitMission:
      spdlog::info("Mission state exited");
      BOOST_VERIFY(false);
      break;
    case DJIWaypointV2MissionStateFinishedMission: {
      spdlog::info("Mission state finished");
      finished = true;
    } break;
    default:
      spdlog::error("Unknown state: {}", state);
      BOOST_VERIFY(false);
  }

  if (finished) {
    self->mission_finished();
    std::lock_guard<std::mutex> lock(self->m_);
    self->execute_commands_.push_back(
        interconnection::command_type::MISSION_FINISHED);
  }

  return OSDK_STAT_OK;
}
