#include "mission_state.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>       // BOOST_VERIFY

void mission_state::start() {
  std::lock_guard<std::mutex> lock(m_);

  BOOST_VERIFY(!is_started_);
  is_started_ = true;

  initial_update_received_ = false;
  state_ = DJI_WAYPOINT_V2_MISSION_STATE_DISCONNECTED;
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

void mission_state::update(T_DjiWaypointV2MissionEventPush event_data) {
  std::lock_guard<std::mutex> lock(m_);

  const unsigned event{ack->event};

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/practice/waypoint-v2-type.html#typedef-struct-t-djiwaypointv2missioneventpush
  if (event == 0x03) { // finish, mission stop event
    spdlog::info("Finish event received");
    BOOST_VERIFY(initial_update_received_);
    BOOST_VERIFY(is_started_);
    is_started_ = false;
    BOOST_VERIFY(state_ != DJI_WAYPOINT_V2_MISSION_STATE_FINISHED_MISSION);
    state_ = DJI_WAYPOINT_V2_MISSION_STATE_FINISHED_MISSION;
    spdlog::info("Updated state: {}", state_name());
  }
}

void mission_state::update(T_DjiWaypointV2MissionStatePush state_data) {
  std::lock_guard<std::mutex> lock(m_);

  // Handle negative values correctly
  const int8_t state{static_cast<int8_t>(state_data.state)};
  const uint16_t waypoint_index{state_data.curWaypointIndex};

  if (!initial_update_received_) {
    initial_update_received_ = true;
    state_ = state;
    BOOST_VERIFY(state_ != DJI_WAYPOINT_V2_MISSION_STATE_FINISHED_MISSION);
    waypoint_index_ = waypoint_index;
    spdlog::info("Starting state: {}, waypoint #{}", state_name(),
                 waypoint_index_);
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

  if (state_ == DJI_WAYPOINT_V2_MISSION_STATE_FINISHED_MISSION) {
    BOOST_VERIFY(is_started_);
    is_started_ = false;
  }
}

const char* mission_state::state_name() const {
  // https://github.com/dji-sdk/Payload-SDK/blob/3.3/psdk_lib/include/dji_waypoint_v2_type.h#L221-L266

  switch (state_) {
    case DJI_WAYPOINT_V2_MISSION_STATE_UNKNOWN:
      return "unknown";
    case DJI_WAYPOINT_V2_MISSION_STATE_DISCONNECTED:
      return "disconnected";
    case DJI_WAYPOINT_V2_MISSION_STATE_READY_TO_EXECUTE:
      return "ready to execute";
    case DJI_WAYPOINT_V2_MISSION_STATE_EXECUTING:
      return "executing";
    case DJI_WAYPOINT_V2_MISSION_STATE_INTERRUPTED:
      return "interrupted";
    case DJI_WAYPOINT_V2_MISSION_STATE_RESUME_AFTER_INTERRUPTED:
      return "resumed";
    case DJI_WAYPOINT_V2_MISSION_STATE_EXIT_MISSION:
      BOOST_VERIFY(false);
      return "exited";
    case DJI_WAYPOINT_V2_MISSION_STATE_FINISHED_MISSION:
      return "finished";
    case mission_unknown_state_7_:
      return "unknown (7)";
    default:
      spdlog::error("Unknown state: {}", state_);
      BOOST_VERIFY(false);
      return "";
  }
}
