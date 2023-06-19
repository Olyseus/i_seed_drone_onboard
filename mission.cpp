#include "mission.h"

#include <spdlog/spdlog.h>

#include <thread>  // std::this_thread

#include "home_altitude.h"
#include "olyseus_verify.h"  // OLYSEUS_VERIFY
#include "utils.h"           // deg2rad

mission::mission() noexcept = default;
mission::~mission() = default;

void mission::init(double lat, double lon) {
  const std::lock_guard lock{m_};

  spdlog::info("Mission parameters: lat({}), lon({})", lat, lon);

  // FIXME (points from polygons)
  // FIXME (action at waypoint?)
  global_waypoints_.clear();

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  global_waypoints_.emplace_back(lat, lon + 0.0001);
  global_waypoints_.emplace_back(lat, lon + 0.0002);
  global_waypoints_.emplace_back(lat, lon + 0.0003);
  global_waypoints_.emplace_back(lat, lon + 0.0004);
#else
  global_waypoints_.emplace_back(lat, lon);
#endif

  mission_state_.init();
}

auto mission::waypoint_reached(float laser_range)
    -> std::pair<waypoint_action, std::size_t> {
  const std::lock_guard lock{m_};

  mission_state_.waypoint_reached();
  const bool is_forward{mission_state_.is_forward()};

  constexpr auto unused_index{std::numeric_limits<std::size_t>::max()};

  std::optional<std::size_t> index{current_waypoint_index(is_forward)};

  if (!index.has_value()) {
    return {waypoint_action::abort, unused_index};
  }

  waypoint& w{global_waypoints_.at(index.value())};

  if (std::abs(laser_range - waypoint::expected_height) > 1.0F) {
    spdlog::info("Bad laser range, waypoint altitude tweak");
    OLYSEUS_VERIFY(is_forward);
    OLYSEUS_VERIFY(w.is_default_altitude());
    w.set_custom_altitude(laser_range);
    OLYSEUS_VERIFY(!w.is_default_altitude());
    return {waypoint_action::restart, unused_index};
  }

  w.set_ready(is_forward);

  return {waypoint_action::ok, index.value()};
}

auto mission::upload_mission_and_start(int32_t event_id) -> bool {
  const std::lock_guard lock{m_};

  spdlog::info("Upload mission and start");

  const bool is_forward{mission_state_.is_forward()};

  // NOLINTNEXTLINE(cert-msc51-cpp, cert-msc32-c)
  srand(time(nullptr));

  T_DjiWayPointV2MissionSettings s;

  // Just a random number
  // NOLINTNEXTLINE(cert-msc30-c, cert-msc50-cpp)
  s.missionID = rand();  // NOLINT(concurrency-mt-unsafe)

  s.repeatTimes = 0;  // execute just once and go home
  s.finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
  constexpr int max_speed{10};
  s.maxFlightSpeed = max_speed;
  s.autoFlightSpeed = 2;
  s.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
  s.gotoFirstWaypointMode =
      DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
  s.actionList.actions = nullptr;
  s.actionList.actionNum = 0;

  waypoints_.clear();

  if (is_forward) {
    for (const waypoint& w : global_waypoints_) {
      if (w.is_forward_ready()) {
        OLYSEUS_VERIFY(waypoints_.empty());
        continue;
      }
      waypoints_.push_back(make_waypoint(w, is_forward));
    }
  } else {
    // NOLINTNEXTLINE(altera-id-dependent-backward-branch)
    for (auto it{global_waypoints_.rbegin()}; it != global_waypoints_.rend();
         ++it) {
      const waypoint& w{*it};
      OLYSEUS_VERIFY(w.is_forward_ready());
      OLYSEUS_VERIFY(!w.is_backward_ready());
      if (w.has_detection()) {
        waypoints_.push_back(make_waypoint(w, is_forward));
      }
    }
  }

  if (waypoints_.empty()) {
    OLYSEUS_VERIFY(event_id == mission_state::internal_event_id);
    OLYSEUS_VERIFY(!is_forward);
    return false;
  }

  if (waypoints_.size() == 1) {
    // Duplicate the last and ignore it when reached
    // Tweak the height to avoid "points are too close" error
    T_DjiWaypointV2 w{waypoints_.back()};
    constexpr double dummy_height{5.0};
    w.relativeHeight += dummy_height;
    waypoints_.push_back(w);
  }
  s.mission = waypoints_.data();

  s.missTotalLen = waypoints_.size();
  OLYSEUS_VERIFY(s.missTotalLen >= 2);
  OLYSEUS_VERIFY(s.missTotalLen <= 65535);

  spdlog::info("Mission start, ID {}", s.missionID);

  T_DjiReturnCode code = DjiWaypointV2_UploadMission(&s);
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    // Assuming that mission can't be uploaded because it's paused
    spdlog::critical("Resume previous mission (?)");
    code = DjiWaypointV2_Resume();
    OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

    while (true) {
      code = DjiWaypointV2_UploadMission(&s);
      if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        break;
      }
      spdlog::critical("Upload failed, waiting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
#endif
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // If DjiWaypointV2_Start failed
  constexpr int wait_ms{1500};
  std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

  code = DjiWaypointV2_Start();
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  mission_state_.start(event_id);

  return true;
}

void mission::set_backward() { mission_state_.set_backward(); }

auto mission::is_forward() const -> bool { return mission_state_.is_forward(); }

auto mission::is_finished() const -> bool {
  return mission_state_.is_finished();
}

auto mission::is_ready() const -> bool { return mission_state_.is_ready(); }

auto mission::is_paused() const -> bool { return mission_state_.is_paused(); }

auto mission::user_cmd_accepted() const -> bool {
  return mission_state_.user_cmd_accepted();
}

void mission::update_event_id(int32_t event_id) {
  mission_state_.update_event_id(event_id);
}

auto mission::update(T_DjiWaypointV2MissionEventPush event_data) -> bool {
  // Note: callback thread, avoid long locks

  return mission_state_.update(event_data);
}

// auto [mission_started, notify]
auto mission::update(T_DjiWaypointV2MissionStatePush state_data)
    -> std::pair<bool, bool> {
  // Note: callback thread, avoid long locks

  return mission_state_.update(state_data);
}

void mission::abort(int32_t event_id) {
  spdlog::info("Mission ABORT");

  // Call it first to block the update callbacks
  mission_state_.abort(event_id);

  const T_DjiReturnCode code{DjiWaypointV2_Stop()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void mission::pause(int32_t event_id) {
  spdlog::info("Pause mission");

  mission_state_.pause(event_id);

  const T_DjiReturnCode code{DjiWaypointV2_Pause()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

void mission::stop(home_altitude& h) {
  h.mission_stop();

  mission_state_.stop();
}

void mission::resume(int32_t event_id) {
  mission_state_.resume(event_id);

  spdlog::info("Mission resume");
  const T_DjiReturnCode code{DjiWaypointV2_Resume()};
  OLYSEUS_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}

auto mission::get_state()
    -> std::pair<int32_t, interconnection::drone_coordinates::state_t> {
  return mission_state_.get_state();
}

auto mission::get_waypoint_copy(std::size_t index) const -> waypoint {
  const std::lock_guard lock{m_};
  mission_state_.waypoint_reached();
  OLYSEUS_VERIFY(index < global_waypoints_.size());
  return global_waypoints_.at(index);
}

void mission::save_detection(std::size_t index,
                             const detection_result& result) {
  const std::lock_guard lock{m_};
  OLYSEUS_VERIFY(index < global_waypoints_.size());
  global_waypoints_.at(index).save_detection(result);
}

auto mission::make_waypoint(const waypoint& w, bool is_forward) const
    -> T_DjiWaypointV2 {
  const double latitude{w.lat()};
  const double longitude{w.lon()};
  const float relative_height{w.altitude()};

  float heading{0.0F};
  std::string heading_str{"auto"};

  if (!is_forward) {
    OLYSEUS_VERIFY(w.has_detection());
    heading = w.heading();
    heading_str = std::to_string(heading);
  }

  spdlog::info("Add waypoint lat({}), lon({}), height({}), heading({})",
               latitude, longitude, relative_height, heading_str);
  T_DjiWaypointV2 p;

  p.latitude = latitude * deg2rad;
  p.longitude = longitude * deg2rad;
  p.relativeHeight = relative_height;

  p.waypointType =
      DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;

  if (is_forward) {
    // Aircraft's heading will always be in the direction of flight
    p.headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
  } else {
    p.headingMode = DJI_WAYPOINT_V2_HEADING_WAYPOINT_CUSTOM;
  }

  p.heading = heading;

  p.config.useLocalCruiseVel = 0;  // set local waypoint's cruise speed
  p.config.useLocalMaxVel = 0;     // set local waypoint's max speed

  constexpr int damping_dist{40};  // cm
  p.dampingDistance = damping_dist;

  p.turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

  // unused
  p.pointOfInterest.positionX = 0.0F;
  p.pointOfInterest.positionY = 0.0F;
  p.pointOfInterest.positionZ = 0.0F;

  constexpr float max_speed{10.0F};
  constexpr float auto_speed{2.0F};

  p.maxFlightSpeed = max_speed;
  p.autoFlightSpeed = auto_speed;

  return p;
}

auto mission::current_waypoint_index(bool is_forward) const
    -> std::optional<std::size_t> {
  std::optional<std::size_t> result;

  if (is_forward) {
    for (std::size_t i{0}; i < global_waypoints_.size(); ++i) {
      const waypoint& w{global_waypoints_[i]};
      if (result.has_value()) {
        OLYSEUS_VERIFY(!w.is_forward_ready());
        continue;
      }

      if (!w.is_forward_ready()) {
        spdlog::info("Global waypoint #{}", i);
        result = i;
      }
    }
  } else {
    // NOLINTNEXTLINE(altera-id-dependent-backward-branch)
    for (std::size_t i{global_waypoints_.size()}; i > 0; --i) {
      const std::size_t index{i - 1};
      const waypoint& w{global_waypoints_[index]};
      if (result.has_value()) {
        OLYSEUS_VERIFY(!w.is_backward_ready());
        continue;
      }

      if (!w.is_backward_ready() && w.has_detection()) {
        spdlog::info("Global waypoint #{}", index);
        result = index;
      }
    }
  }

  return result;
}
