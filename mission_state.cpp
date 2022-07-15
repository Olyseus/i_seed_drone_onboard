#include "mission_state.h"

void mission_state::start() {
  std::lock_guard<std::mutex> lock(m_);

  BOOST_VERIFY(!is_started_);
  is_started_ = true;

  initial_update_received_ = false;
  state_ = 0;
  waypoint_index_ = 0;
}

void mission_state::finish() {
  std::lock_guard<std::mutex> lock(m_);

  BOOST_VERIFY(is_started_);
  is_started_ = false;
}

bool mission_state::is_started() const {
  std::lock_guard<std::mutex> lock(m_);
  return is_started_;
}

bool mission_state::is_disconnected() const {
  std::lock_guard<std::mutex> lock(m_);
  BOOST_VERIFY(is_started_);
  BOOST_VERIFY(initial_update_received_);
  return state_ == mission_unknown_state_7_ || state_ == DJIWaypointV2MissionStateDisconnected;
}

void mission_state::update(const DJI::OSDK::MissionStatePushAck* ack) {
  std::lock_guard<std::mutex> lock(m_);

  // Handle negative values correctly
  const int8_t state{static_cast<int8_t>(ack->data.state)};
  const uint16_t waypoint_index{ack->data.curWaypointIndex};

  if (!initial_update_received_) {
    initial_update_received_ = true;
    state_ = state;
    BOOST_VERIFY(state_ != DJIWaypointV2MissionStateFinishedMission);
    waypoint_index_ = waypoint_index;
    spdlog::info("Starting state: {}, waypoint #{}", state_name(), waypoint_index_);
    return;
  }

  if (state_ != state) {
    state_ = state;
    spdlog::info("Updated state: {}", state_name());
  }

  if (waypoint_index_ != waypoint_index) {
    waypoint_index_ = waypoint_index;
    spdlog::info("Waypoint #{}", waypoint_index_);
  }

  if (state_ == DJIWaypointV2MissionStateFinishedMission) {
    BOOST_VERIFY(is_started_);
    is_started_ = false;
  }
}

const char* mission_state::state_name() const {
  switch (state_) {
    case DJIWaypointV2MissionStateUnWaypointActionActuatorknown:
      return "unknown";
    case DJIWaypointV2MissionStateDisconnected:
      return "disconnected";
    case DJIWaypointV2MissionStateReadyToExecute:
      return "ready to execute";
    case DJIWaypointV2MissionStateExecuting:
      return "executing";
    case DJIWaypointV2MissionStateInterrupted:
      return "interrupted";
    case DJIWaypointV2MissionStateResumeAfterInterrupted:
      return "resumed";
    case DJIWaypointV2MissionStateExitMission:
      BOOST_VERIFY(false);
      return "exited";
    case DJIWaypointV2MissionStateFinishedMission:
      return "finished";
    case mission_unknown_state_7_:
      return "unknown (7)";
    default:
      spdlog::error("Unknown state: {}", state);
      BOOST_VERIFY(false);
      return "";
  }
}
