#include "mission_state.h"

#include <spdlog/spdlog.h>

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

void mission_state::mission_path_ready(int32_t event_id) {
  const std::lock_guard lock{m_};

  user_pause_ = false;

  OLYSEUS_VERIFY(global_state_ == ready);
  global_state_ = path_data;

  OLYSEUS_VERIFY(!next_state_.has_value());
  OLYSEUS_VERIFY(event_id != internal_event_id);

  OLYSEUS_VERIFY(event_id == event_id_ + 1);
  event_id_ = event_id;
}

void mission_state::mission_path_cancel(int32_t event_id) {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(global_state_ == path);
  global_state_ = ready;

  OLYSEUS_VERIFY(!next_state_.has_value());
  OLYSEUS_VERIFY(event_id != internal_event_id);

  OLYSEUS_VERIFY(event_id == event_id_ + 1);
  event_id_ = event_id;
}

auto mission_state::mission_path_available() const -> bool {
  const std::lock_guard lock{m_};

  return global_state_ == path;
}

void mission_state::init() {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(global_state_ == path);
  global_state_ = forward_wait_start;
}

void mission_state::start(int32_t event_id) {
  const std::lock_guard lock{m_};

  if (global_state_ == forward_wait_start) {
    global_state_ = forward_wait_update;
  } else {
    OLYSEUS_VERIFY(global_state_ == backward_wait_start);
    global_state_ = backward_wait_update;
  }

  set_next_state(interconnection::drone_info::EXECUTING, event_id);

  already_executed_.clear();
}

void mission_state::pause(int32_t event_id) {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(!user_pause_);
  user_pause_ = true;

  OLYSEUS_VERIFY(global_state_ == forward_executing ||
                 global_state_ == backward_executing);

  set_next_state(interconnection::drone_info::PAUSED, event_id);
}

void mission_state::stop() {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(global_state_ == forward_wait_start ||
                 global_state_ == backward_finished ||
                 global_state_ == backward_wait_start);
  global_state_ = ready;
}

void mission_state::abort(int32_t event_id) {
  const std::lock_guard lock{m_};

  if (global_state_ == forward_executing) {
    global_state_ = forward_wait_start;
  } else {
    OLYSEUS_VERIFY(global_state_ == backward_executing);
    global_state_ = backward_wait_start;
  }

  // 'event_id' for internal mission restart is invalid,
  // if 'event_id' is valid, it means user aborted the mission and next
  // state is 'READY'
  set_next_state(interconnection::drone_info::READY, event_id);
}

void mission_state::resume(int32_t event_id) {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(user_pause_);
  user_pause_ = false;

  OLYSEUS_VERIFY(global_state_ == forward_executing ||
                 global_state_ == backward_executing);
  OLYSEUS_VERIFY(event_id != internal_event_id);

  set_next_state(interconnection::drone_info::EXECUTING, event_id);
}

void mission_state::update_event_id(int32_t event_id) {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(global_state_ == ready);
  OLYSEUS_VERIFY(!next_state_.has_value());

  OLYSEUS_VERIFY(event_id == event_id_ + 1);
  event_id_ = event_id;
}

void mission_state::waypoint_reached() const {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(global_state_ == forward_executing ||
                 global_state_ == backward_executing);
}

auto mission_state::is_forward() const -> bool {
  const std::lock_guard lock{m_};

  switch (global_state_) {
    case forward_wait_start:
    case forward_executing:
    case forward_finished:
      return true;
    case backward_wait_start:
    case backward_executing:
    case backward_finished:
      return false;
    case forward_wait_update:
    case backward_wait_update:
    case ready:
    case path_data:
    case path:
    default:
      OLYSEUS_UNREACHABLE;
  }
}

void mission_state::set_backward() {
  const std::lock_guard lock{m_};

  OLYSEUS_VERIFY(global_state_ == forward_finished ||
                 global_state_ == forward_wait_start);
  global_state_ = backward_wait_start;
}

auto mission_state::is_finished() const -> bool {
  const std::lock_guard lock{m_};

  return global_state_ == forward_finished ||
         global_state_ == backward_finished;
}

auto mission_state::is_ready() const -> bool {
  const std::lock_guard lock{m_};

  return global_state_ == ready;
}

auto mission_state::is_paused() const -> bool {
  const std::lock_guard lock{m_};

  if (global_state_ != backward_executing &&
      global_state_ != forward_executing) {
    return false;
  }

  if (next_state_.has_value() &&
      next_state_.value() == interconnection::drone_info::PAUSED) {
    // PAUSE command was received from user but mission state hasn't updated yet
    return true;
  }

  return state_ == pause_state;
}

auto mission_state::user_cmd_accepted() const -> bool {
  const std::lock_guard lock{m_};

  return global_state_ == forward_executing ||
         global_state_ == backward_executing || global_state_ == ready;
}

auto mission_state::update(T_DjiWaypointV2MissionEventPush event_data) -> bool {
  const std::lock_guard lock{m_};

  if (!check_updates()) {
    return false;
  }

  const unsigned event{event_data.event};

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/practice/waypoint-v2-type.html#typedef-struct-t-djiwaypointv2missioneventpush
  if (event == 0x03) {  // finish, mission stop event
    spdlog::info("Finish event received");

    OLYSEUS_VERIFY(global_state_ != forward_wait_update);
    OLYSEUS_VERIFY(global_state_ != backward_wait_update);

    if (global_state_ == forward_executing) {
      global_state_ = forward_finished;
    } else {
      OLYSEUS_VERIFY(global_state_ == backward_executing);
      global_state_ = backward_finished;
    }

    OLYSEUS_VERIFY(state_ != exit_mission);
    state_ = exit_mission;
    spdlog::info("Updated state: {}", state_name());

    return true;
  }

  return false;
}

// @return auto [mission_started, notify]
auto mission_state::update(T_DjiWaypointV2MissionStatePush state_data)
    -> std::pair<bool, bool> {
  spdlog::debug("update: T_DjiWaypointV2MissionStatePush.state = {}",
                static_cast<unsigned>(state_data.state));
  const std::lock_guard lock{m_};

  const bool mission_started{check_updates()};
  bool notify{false};

  if (!mission_started) {
    return {mission_started, notify};
  }

  const uint16_t waypoint_index{state_data.curWaypointIndex};

  const bool forward_wait = (global_state_ == forward_wait_update);
  const bool backward_wait = (global_state_ == backward_wait_update);

  if (forward_wait || backward_wait) {
    if (forward_wait) {
      global_state_ = forward_executing;
    } else {
      OLYSEUS_VERIFY(backward_wait);
      global_state_ = backward_executing;
    }
    state_ = state_data.state;
    OLYSEUS_VERIFY(state_ != exit_mission);
    waypoint_index_ = waypoint_index;
    spdlog::info("Starting state: {}, waypoint #{}", state_name(),
                 waypoint_index_);
    return {mission_started, notify};
  }

  if (state_ != state_data.state || waypoint_index_ != waypoint_index) {
    state_ = state_data.state;
    waypoint_index_ = waypoint_index;
    spdlog::info("State: {}, waypoint #{}", state_name(), waypoint_index_);
    if (state_ == execute_flying_route_mission) {
      auto [it, inserted] = already_executed_.insert(waypoint_index_);
      OLYSEUS_VERIFY(it != already_executed_.end());
      OLYSEUS_VERIFY(*it == waypoint_index_);
      notify = inserted;
    }
  }

  if (state_ == exit_mission) {
    OLYSEUS_VERIFY(!notify);
    notify = true;

    if (global_state_ == forward_executing) {
      global_state_ = forward_finished;
    } else {
      OLYSEUS_VERIFY(global_state_ == backward_executing);
      global_state_ = backward_finished;
    }

    return {mission_started, notify};
  }

  return {mission_started, notify};
}

auto mission_state::get_state()
    -> std::pair<int32_t, interconnection::drone_info::state_t> {
  const std::lock_guard lock{m_};

  if (global_state_ == path_data) {
    OLYSEUS_VERIFY(!next_state_.has_value());
    global_state_ = path;
    return {event_id_, interconnection::drone_info::PATH_DATA};
  }

  if (global_state_ == path) {
    OLYSEUS_VERIFY(!next_state_.has_value());
    return {event_id_, interconnection::drone_info::PATH};
  }

  if (global_state_ == ready) {
    return get_state(interconnection::drone_info::READY);
  }

  if (global_state_ != backward_executing &&
      global_state_ != forward_executing) {
    return get_state(interconnection::drone_info::WAITING);
  }

  switch (state_) {
    case ground_station_not_start:
    case exit_mission:
    case end_of_waypoint_mission:
    case mission_prepared:
      return get_state(interconnection::drone_info::WAITING);
    case pause_state:
      if (user_pause_) {
        return get_state(interconnection::drone_info::PAUSED);
      } else {
        // Waypoint reached, user visible state should be EXECUTING
        return get_state(interconnection::drone_info::EXECUTING);
      }
    case enter_mission:
    case execute_flying_route_mission:
    case enter_mission_after_ending_pause:
      return get_state(interconnection::drone_info::EXECUTING);
    default:
      spdlog::error("Unknown state: {}", static_cast<unsigned>(state_));
      OLYSEUS_UNREACHABLE;
      return get_state(interconnection::drone_info::EXECUTING);
  }
}

auto mission_state::get_state(interconnection::drone_info::state_t new_state)
    -> std::pair<int32_t, interconnection::drone_info::state_t> {
  if (next_state_.has_value()) {
    if (next_state_.value() == new_state) {
      next_state_.reset();
      OLYSEUS_VERIFY(!next_state_.has_value());
      OLYSEUS_VERIFY(event_id_ + 1 == next_event_id_);
      event_id_ = next_event_id_;
      spdlog::debug("Event id updated: {}", event_id_);
    } else {
      spdlog::debug("State doesn't match, ignored");
    }
  }

  return {event_id_, new_state};
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
      OLYSEUS_UNREACHABLE;
      return "";
  }
}

auto mission_state::check_updates() const -> bool {
  switch (global_state_) {
    case ready:
    case path_data:
    case path:
    case forward_wait_start:
    case forward_finished:
    case backward_wait_start:
    case backward_finished:
      return false;
    case forward_wait_update:
    case forward_executing:
    case backward_wait_update:
    case backward_executing:
      return true;
    default:
      OLYSEUS_UNREACHABLE;
  }
}

void mission_state::set_next_state(interconnection::drone_info::state_t s,
                                   int32_t event_id) {
  OLYSEUS_VERIFY(s != interconnection::drone_info::WAITING);

  if (event_id == internal_event_id) {
    return;
  }

  OLYSEUS_VERIFY(!next_state_.has_value());
  next_state_ = s;

  next_event_id_ = event_id;
  OLYSEUS_VERIFY(next_event_id_ == event_id_ + 1);
}
