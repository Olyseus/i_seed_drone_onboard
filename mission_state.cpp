#include "mission_state.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY

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

auto mission_state::is_started() const -> bool {
  std::lock_guard<std::mutex> lock(m_);
  return is_started_;
}

auto mission_state::update(T_DjiWaypointV2MissionEventPush event_data) -> bool {
  std::lock_guard<std::mutex> lock(m_);

  if (!is_started_) {
    return false;
  }

  const unsigned event{event_data.event};

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/practice/waypoint-v2-type.html#typedef-struct-t-djiwaypointv2missioneventpush
  if (event == 0x03) {  // finish, mission stop event
    spdlog::info("Finish event received");
    BOOST_VERIFY(initial_update_received_);
    BOOST_VERIFY(is_started_);
    is_started_ = false;
    BOOST_VERIFY(state_ != exit_mission);
    state_ = exit_mission;
    spdlog::info("Updated state: {}", state_name());

    return true;
  }

  return false;
}

// @return auto [mission_started, notify, notify_finished]
auto mission_state::update(T_DjiWaypointV2MissionStatePush state_data)
    -> std::tuple<bool, bool, bool> {
  spdlog::debug("update: T_DjiWaypointV2MissionStatePush.state = {}",
                static_cast<unsigned>(state_data.state));
  std::lock_guard<std::mutex> lock(m_);

  const bool mission_started{is_started_};
  bool notify{false};
  bool notify_finished{false};

  if (!mission_started) {
    return {mission_started, notify, notify_finished};
  }

  const uint16_t waypoint_index{state_data.curWaypointIndex};

  if (!initial_update_received_) {
    initial_update_received_ = true;
    state_ = state_data.state;
    BOOST_VERIFY(state_ != exit_mission);
    waypoint_index_ = waypoint_index;
    spdlog::info("Starting state: {}, waypoint #{}", state_name(),
                 waypoint_index_);
    return {mission_started, notify, notify_finished};
  }

  if (state_ != state_data.state || waypoint_index_ != waypoint_index) {
    state_ = state_data.state;
    waypoint_index_ = waypoint_index;
    spdlog::info("State: {}, waypoint #{}", state_name(), waypoint_index_);
    if (state_ == execute_flying_route_mission) {
      auto [it, inserted] = already_executed_.insert(waypoint_index_);
      BOOST_VERIFY(it != already_executed_.end());
      BOOST_VERIFY(*it == waypoint_index_);
      notify = inserted;
    }
  }

  if (state_ == exit_mission) {
    BOOST_VERIFY(!notify);
    notify = true;
    BOOST_VERIFY(is_started_);
    is_started_ = false;
    notify_finished = true;
    return {mission_started, notify, notify_finished};
  }

  return {mission_started, notify, notify_finished};
}

auto mission_state::get_state() const
    -> interconnection::drone_coordinates::state_t {
  std::lock_guard<std::mutex> lock(m_);

  if (!is_started_) {
    return interconnection::drone_coordinates::READY;
  }

  if (!initial_update_received_) {
    return interconnection::drone_coordinates::WAITING;
  }

  switch (state_) {
    case ground_station_not_start:
      return interconnection::drone_coordinates::WAITING;
    case pause_state:
      return interconnection::drone_coordinates::PAUSED;
    case mission_prepared:
    case enter_mission:
    case execute_flying_route_mission:
    case enter_mission_after_ending_pause:
    case exit_mission:
    case end_of_waypoint_mission:
      return interconnection::drone_coordinates::EXECUTING;
    default:
      spdlog::error("Unknown state: {}", static_cast<unsigned>(state_));
      BOOST_VERIFY(false);
      return interconnection::drone_coordinates::EXECUTING;
  }
}

auto mission_state::state_name() const -> const char* {
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
    case end_of_waypoint_mission:
      return "end of waypoint mission";
    default:
      spdlog::error("Unknown state: {}", static_cast<unsigned>(state_));
      BOOST_VERIFY(false);
      return "";
  }
}
