#include "mission_state.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>       // BOOST_VERIFY

void mission_state::start() {
  std::lock_guard<std::mutex> lock(m_);

  BOOST_VERIFY(!is_started_);
  is_started_ = true;

  initial_update_received_ = false;
  state_ = ground_station_not_start;
  waypoint_index_ = 0;

  already_executed_.clear();
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

  const unsigned event{event_data.event};

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/practice/waypoint-v2-type.html#typedef-struct-t-djiwaypointv2missioneventpush
  if (event == 0x03) { // finish, mission stop event
    spdlog::info("Finish event received");
    BOOST_VERIFY(initial_update_received_);
    BOOST_VERIFY(is_started_);
    is_started_ = false;
    BOOST_VERIFY(state_ != exit_mission);
    state_ = exit_mission;
    spdlog::info("Updated state: {}", state_name());
  }
}

uint16_t mission_state::update(T_DjiWaypointV2MissionStatePush state_data) {
  spdlog::debug("update: T_DjiWaypointV2MissionStatePush.state = {}", static_cast<unsigned>(state_data.state));
  std::lock_guard<std::mutex> lock(m_);

  const uint16_t waypoint_index{state_data.curWaypointIndex};

  if (!initial_update_received_) {
    initial_update_received_ = true;
    state_ = state_data.state;
    BOOST_VERIFY(state_ != exit_mission);
    waypoint_index_ = waypoint_index;
    spdlog::info("Starting state: {}, waypoint #{}", state_name(),
                 waypoint_index_);
    return invalid_waypoint;
  }

  bool run_action{false};

  if (state_ != state_data.state || waypoint_index_ != waypoint_index) {
    state_ = state_data.state;
    waypoint_index_ = waypoint_index;
    spdlog::info("State: {}, waypoint #{}", state_name(), waypoint_index_);
    if (state_ == execute_flying_route_mission) {
      auto [it, inserted] = already_executed_.insert(waypoint_index_);
      BOOST_VERIFY(it != already_executed_.end());
      BOOST_VERIFY(*it == waypoint_index_);
      run_action = inserted;
    }
  }

  if (state_ == exit_mission) {
    BOOST_VERIFY(!run_action);
    BOOST_VERIFY(is_started_);
    is_started_ = false;
  }

  if (run_action) {
    BOOST_VERIFY(waypoint_index_ != invalid_waypoint);
    return waypoint_index_;
  } else {
    return invalid_waypoint;
  }
}

const char* mission_state::state_name() const {
  switch (state_) {
    case ground_station_not_start:
      return "ground station not start";
    case mission_prepared:
      return "mission prepared";
    case enter_mission:
      return "enter mission";
    case execute_flying_route_mission:
      return "execute flying route mission";
    case pause_state:
      return "pause state";
    case enter_mission_after_ending_pause:
      return "enter mission after ending pause";
    case exit_mission:
      return "exit mission";
    default:
      spdlog::error("Unknown state: {}", static_cast<unsigned>(state_));
      BOOST_VERIFY(false);
      return "";
  }
}
