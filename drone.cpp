#include "drone.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>
#include <dji_linux_helpers.hpp>  // LinuxSetup
#include <future>

drone::drone(int argc, char** argv) {
  spdlog::info("Setup for Linux");
  constexpr bool enable_advanced_sensing{true};
  linux_setup_.reset(new LinuxSetup(argc, argv, enable_advanced_sensing));

  constexpr int freq{5};
  DJI::OSDK::Telemetry::TopicName topic_list[] = {
      DJI::OSDK::Telemetry::TOPIC_QUATERNION,
      DJI::OSDK::Telemetry::TOPIC_GPS_FUSED};
  constexpr std::size_t num_topic{sizeof(topic_list) / sizeof(topic_list[0])};
  constexpr bool enable_timestamp{false};

  Vehicle* vehicle{linux_setup_->getVehicle()};
  BOOST_VERIFY(vehicle);

  const bool pkg_status = vehicle->subscribe->initPackageFromTopicList(
      pkg_index, num_topic, topic_list, enable_timestamp, freq);
  BOOST_VERIFY(pkg_status);

  constexpr int response_timeout{1};
  ACK::ErrorCode subscribe_status =
      vehicle->subscribe->startPackage(pkg_index, response_timeout);
  BOOST_VERIFY(ACK::getError(subscribe_status) == ACK::SUCCESS);

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
  if (linux_setup_.get()) {
    Vehicle* vehicle{linux_setup_->getVehicle()};
    if (vehicle) {
      constexpr int response_timeout{1};
      vehicle->subscribe->removePackage(pkg_index, response_timeout);
    }
  }
}

void drone::start() {
  spdlog::info("Protocol version: {}", protocol_version);
  spdlog::info("Command bytes size: {}", command_bytes_size_);

  while (true) {
    connection_closed_ = false;

    std::unique_ptr<MopServer> server{new MopServer()};

    spdlog::info("Creating channel {}", channel_id);
    MopErrCode error{
        server->accept(channel_id, MOP::PipelineType::RELIABLE, pipeline_)};
    if (error == MOP_NOTREADY) {
      spdlog::info("Retry");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }
    spdlog::info("Accept code: {}", error);
    BOOST_VERIFY(error == MOP_PASSED);
    BOOST_VERIFY(pipeline_ != nullptr);

    std::future<void> read_future{
        std::async(std::launch::async, &drone::read_job, this)};
    std::future<void> write_future{
        std::async(std::launch::async, &drone::write_job, this)};

    read_future.get();
    write_future.get();

    error = server->close(channel_id);
    BOOST_VERIFY(error == MOP_PASSED);
  }
}

void drone::read_job() {
  spdlog::info("Read job started");

  std::string buffer;
  buffer.resize(command_bytes_size_);

  while (true) {
    if (connection_closed_) {
      return;
    }

    BOOST_VERIFY(command_bytes_size_ == buffer.size());
    if (!read_data(&buffer)) {
      continue;
    }
    interconnection::command_type command;
    const bool ok{command.ParseFromString(buffer)};
    BOOST_VERIFY(ok);

    Vehicle* vehicle{linux_setup_->getVehicle()};
    BOOST_VERIFY(vehicle);

    switch (command.type()) {
      case interconnection::command_type::PING: {
        std::lock_guard<std::mutex> lock(m_);
        execute_commands_.push_back(command.type());
      } break;
      case interconnection::command_type::MISSION_START: {
        std::string buffer;
        buffer.resize(pin_coordinates_bytes_size_);
        if (!read_data(&buffer)) {
          continue;
        }

        if (mission_is_started_) {
          spdlog::info("Mission resume");
          constexpr int timeout{10};
          ErrorCode::ErrorCodeType ret =
              vehicle->waypointV2Mission->resume(timeout);
          BOOST_VERIFY(ret == ErrorCode::SysCommonErr::Success);
          // FIXME (verify mission state)
          continue;
        }

        interconnection::pin_coordinates pin_coordinates;
        const bool ok{pin_coordinates.ParseFromString(buffer)};
        BOOST_VERIFY(ok);

        const double lat{pin_coordinates.latitude()};
        const double lon{pin_coordinates.longitude()};

        spdlog::info("Mission start: lat({}), lon({})", lat, lon);

        constexpr int timeout{10};
        ACK::ErrorCode res{vehicle->control->obtainCtrlAuthority(timeout)};
        BOOST_VERIFY(ACK::getError(res) == ACK::SUCCESS);

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

        ErrorCode::ErrorCodeType ret{
            vehicle->waypointV2Mission->init(&s, timeout)};
        BOOST_VERIFY(ret == ErrorCode::SysCommonErr::Success);

        ret = vehicle->waypointV2Mission->uploadMission(timeout);
        BOOST_VERIFY(ret == ErrorCode::SysCommonErr::Success);

        ret = vehicle->waypointV2Mission->start(timeout);
        BOOST_VERIFY(ret == ErrorCode::SysCommonErr::Success);

        vehicle->waypointV2Mission->RegisterMissionStateCallback(
            this, update_mission_state);

        mission_is_started_ = true;
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_PAUSE: {
        spdlog::info("Mission pause");
        constexpr int timeout{10};
        ErrorCode::ErrorCodeType ret =
            vehicle->waypointV2Mission->pause(timeout);
        BOOST_VERIFY(ret == ErrorCode::SysCommonErr::Success);
        // FIXME (verify mission state)
      } break;
      case interconnection::command_type::MISSION_ABORT: {
        spdlog::info("Mission abort");
        constexpr int timeout{10};
        ErrorCode::ErrorCodeType ret =
            vehicle->waypointV2Mission->stop(timeout);
        BOOST_VERIFY(ret == ErrorCode::SysCommonErr::Success);
        // FIXME (verify mission state)
        mission_finished();
      } break;
      default:
        BOOST_VERIFY(false);
    }
  }
}

void drone::write_job() {
  spdlog::info("Write job started");

  std::optional<interconnection::command_type::command_t> command;

  while (true) {
    if (connection_closed_) {
      return;
    }
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

    Vehicle* vehicle{linux_setup_->getVehicle()};
    BOOST_VERIFY(vehicle);

    switch (command.value()) {
      case interconnection::command_type::PING: {
        spdlog::info("Execute PING command");
        if (!send_command(interconnection::command_type::PING)) {
          continue;
        }
        break;
      }
      case interconnection::command_type::MISSION_FINISHED: {
        spdlog::info("Execute MISSION_FINISHED command");
        if (!send_command(interconnection::command_type::MISSION_FINISHED)) {
          continue;
        }
      } break;
      case interconnection::command_type::DRONE_COORDINATES: {
        if (!send_command(interconnection::command_type::DRONE_COORDINATES)) {
          continue;
        }

        DJI::OSDK::Telemetry::Quaternion quaternion{
            vehicle->subscribe
                ->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>()};
        DJI::OSDK::Telemetry::GPSFused global{
            vehicle->subscribe
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

        if (!write_data(buffer)) {
          return;
        }

        spdlog::info("Drone coordinates sent: lat:{}, lon:{}, head:{}",
                     latitude, longitude, heading);
        break;
      }
      default:
        BOOST_VERIFY(false);
    }

    {
      std::lock_guard<std::mutex> lock(m_);
      execute_commands_.pop_front();
    }
  }
}

bool drone::send_command(
    interconnection::command_type::command_t command_type) {
  interconnection::command_type command;
  command.set_type(command_type);
  command.set_version(protocol_version);
  std::string buffer;
  const bool ok{command.SerializeToString(&buffer)};
  BOOST_VERIFY(ok);
  BOOST_VERIFY(buffer.size() == command_bytes_size_);

  return write_data(buffer);
}

bool drone::read_data(std::string* buffer) {
  BOOST_VERIFY(buffer != nullptr);
  BOOST_VERIFY(buffer->size() > 0);

  char* char_buffer{buffer->data()};
  static_assert(sizeof(char) == sizeof(uint8_t));
  uint8_t* recv_buf{reinterpret_cast<uint8_t*>(char_buffer)};
  MopPipeline::DataPackType read_pack = {recv_buf,
                                         static_cast<uint32_t>(buffer->size())};
  uint32_t len{0};

  while (true) {
    MopErrCode result{pipeline_->recvData(read_pack, &len)};
    spdlog::info("Read data code: {} (size: {})", result, len);

    if (result == MOP_TIMEOUT) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    if (result == MOP_CONNECTIONCLOSE) {
      spdlog::info("Read connection closed");
      connection_closed_ = true;
      return false;
    }

    if (result == MOP_NOMEM) {
      spdlog::info("No memory for read");
      connection_closed_ = true;
      return false;
    }

    BOOST_VERIFY(result == MOP_PASSED);
    BOOST_VERIFY(len == buffer->size());
    return true;
  }
}

bool drone::write_data(std::string& buffer) {
  BOOST_VERIFY(buffer->size() > 0);

  char* char_buffer{buffer.data()};
  static_assert(sizeof(char) == sizeof(uint8_t));
  uint8_t* send_buf{reinterpret_cast<uint8_t*>(char_buffer)};
  MopPipeline::DataPackType req_pack = {send_buf,
                                        static_cast<uint32_t>(buffer.size())};
  uint32_t len{0};

  while (true) {
    MopErrCode result{pipeline_->sendData(req_pack, &len)};
    spdlog::info("Write data code: {} (size: {})", result, len);

    if (result == MOP_TIMEOUT) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    if (result == MOP_CONNECTIONCLOSE) {
      spdlog::info("Write connection closed");
      connection_closed_ = true;
      return false;
    }

    if (result == MOP_NOMEM) {
      spdlog::info("No memory for write");
      connection_closed_ = true;
      return false;
    }
  }

  BOOST_VERIFY(result == MOP_PASSED);
  BOOST_VERIFY(len == buffer.size());
  return true;
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

  Vehicle* vehicle{linux_setup_->getVehicle()};
  BOOST_VERIFY(vehicle);

  constexpr int timeout{10};
  ACK::ErrorCode res{vehicle->control->releaseCtrlAuthority(timeout)};
  if (ACK::getError(res) == ACK::SUCCESS) {
    spdlog::warn("Authority not released: {}", ACK::getError(res));
  }
}

E_OsdkStat drone::update_mission_state(T_CmdHandle* cmd_handle,
                                       const T_CmdInfo* cmd_info,
                                       const uint8_t* cmd_data,
                                       void* user_data) {
  BOOST_VERIFY(user_data != nullptr);
  auto* self{static_cast<drone*>(user_data)};

  BOOST_VERIFY(cmd_data != nullptr);
  auto* ack{reinterpret_cast<const DJI::OSDK::MissionStatePushAck*>(cmd_data)};
  auto state{
      static_cast<DJI::OSDK::DJIWaypointV2MissionState>(ack->data.state)};

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
      spdlog::info("Mission state executing");
      break;
    case DJIWaypointV2MissionStateInterrupted:
      spdlog::info("Mission state interrupted");
      // Interrupted by second smart controller while
      // asking for a landing confirmation
      finished = true;
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
