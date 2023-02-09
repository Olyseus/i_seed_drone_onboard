#include "mission.h"

#include <boost/assert.hpp> // BOOST_VERIFY
#include <spdlog/spdlog.h>
#include <thread> // std::this_thread

#include "mission_state.h" // mission_state_
#include "utils.h" // deg2rad

mission::mission(mission_state& m) : mission_state_(m) {}
mission::~mission() = default;

void mission::init(double lat, double lon) {
  std::lock_guard<std::mutex> lock(m_);

  spdlog::info("Mission parameters: lat({}), lon({})", lat, lon);

  // FIXME (points from polygons)
  // FIXME (action at waypoint?)
  global_waypoints_.clear();
  global_waypoints_.emplace_back(lat, lon + 0.0001);
  global_waypoints_.emplace_back(lat, lon + 0.0002);
  global_waypoints_.emplace_back(lat, lon + 0.0003);
  global_waypoints_.emplace_back(lat, lon + 0.0004);

  is_forward_ = true;
}

auto mission::waypoint_reached(double laser_range, std::size_t* waypoint_index) -> waypoint_action {
  std::lock_guard<std::mutex> lock(m_);

  std::optional<std::size_t> index{current_waypoint_index()};

  if (!index.has_value()) {
    return waypoint_action::abort;
  }

  BOOST_VERIFY(waypoint_index != nullptr);
  *waypoint_index = index.value();

  waypoint& w{global_waypoints_.at(index.value())};

  if (std::abs(laser_range - waypoint::expected_height) > 1.0) {
    spdlog::info("Bad laser range, waypoint altitude tweak");
    BOOST_VERIFY(is_forward_);
    BOOST_VERIFY(w.is_default_altitude());
    w.set_custom_altitude(laser_range);
    BOOST_VERIFY(!w.is_default_altitude());
    return waypoint_action::restart;
  }

  w.set_ready(is_forward_);

  return waypoint_action::ok;
}

void mission::upload_mission_and_start() {
  std::lock_guard<std::mutex> lock(m_);

  spdlog::info("Upload mission and start");

  srand(time(nullptr));

  T_DjiWayPointV2MissionSettings s;
  s.missionID = rand();  // Just a random number
  s.repeatTimes = 0;     // execute just once and go home
  s.finishedAction = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
  s.maxFlightSpeed = 10;
  s.autoFlightSpeed = 2;
  s.actionWhenRcLost = DJI_WAYPOINT_V2_MISSION_KEEP_EXECUTE_WAYPOINT_V2;
  s.gotoFirstWaypointMode = DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
  s.actionList.actions = nullptr;
  s.actionList.actionNum = 0;

  waypoints_.clear();

  if (is_forward_) {
    for (const waypoint& w : global_waypoints_) {
      if (w.is_forward_ready()) {
        BOOST_VERIFY(waypoints_.empty());
        continue;
      }
      waypoints_.push_back(make_waypoint(w));
    }
  }
  else {
    for (auto it{global_waypoints_.rbegin()}; it != global_waypoints_.rend(); ++it) {
      const waypoint& w{*it};
      BOOST_VERIFY(w.is_forward_ready());
      BOOST_VERIFY(!w.is_backward_ready());
      if (w.has_detection()) {
        waypoints_.push_back(make_waypoint(w));
      }
    }
  }

  BOOST_VERIFY(!waypoints_.empty());

  if (waypoints_.size() == 1) {
    // Duplicate the last and ignore it when reached
    // Tweak the height to avoid "points are too close" error
    T_DjiWaypointV2 w{waypoints_.back()};
    w.relativeHeight += 5.0;
    waypoints_.push_back(w);
  }
  s.mission = waypoints_.data();

  s.missTotalLen = waypoints_.size();
  BOOST_VERIFY(s.missTotalLen >= 2);
  BOOST_VERIFY(s.missTotalLen <= 65535);

  spdlog::info("Mission start, ID {}", s.missionID);

  T_DjiReturnCode code = DjiWaypointV2_UploadMission(&s);
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    // Assuming that mission can't be uploaded because it's paused
    spdlog::critical("Resume previous mission (?)");
    code = DjiWaypointV2_Resume();
    BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

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
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  // If DjiWaypointV2_Start failed
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  code = DjiWaypointV2_Start();
  BOOST_VERIFY(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

  mission_state_.start();
}

void mission::set_backward() {
  std::lock_guard<std::mutex> lock(m_);
  BOOST_VERIFY(is_forward_);
  is_forward_ = false;
}

auto mission::is_forward() const -> bool {
  std::lock_guard<std::mutex> lock(m_);
  return is_forward_;
}

auto mission::get_waypoint_copy(std::size_t index) const -> waypoint {
  std::lock_guard<std::mutex> lock(m_);
  BOOST_VERIFY(index < global_waypoints_.size());
  return global_waypoints_.at(index);
}

void mission::save_detection(std::size_t index, const detection_result& result) {
  std::lock_guard<std::mutex> lock(m_);
  BOOST_VERIFY(index < global_waypoints_.size());
  global_waypoints_.at(index).save_detection(result);
}

T_DjiWaypointV2 mission::make_waypoint(const waypoint& w) {
  const double latitude{w.lat()};
  const double longitude{w.lon()};
  const double relative_height{w.altitude()};

  double heading{0.0};
  std::string heading_str{"auto"};

  if (!is_forward_) {
    BOOST_VERIFY(w.has_detection());
    heading = w.heading();
    heading_str = std::to_string(heading);
  }

  spdlog::info("Add waypoint lat({}), lon({}), height({}), heading({})", latitude, longitude, relative_height, heading_str);
  T_DjiWaypointV2 p;

  p.latitude = latitude * deg2rad;
  p.longitude = longitude * deg2rad;
  p.relativeHeight = relative_height;

  p.waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;

  if (is_forward_) {
    // Aircraft's heading will always be in the direction of flight
    p.headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
  } else {
    p.headingMode = DJI_WAYPOINT_V2_HEADING_WAYPOINT_CUSTOM;
  }

  p.heading = heading;

  p.config.useLocalCruiseVel = 0;  // set local waypoint's cruise speed
  p.config.useLocalMaxVel = 0;     // set local waypoint's max speed

  p.dampingDistance = 40;  // cm

  p.turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

  // unused
  p.pointOfInterest.positionX = 0.0F;
  p.pointOfInterest.positionY = 0.0F;
  p.pointOfInterest.positionZ = 0.0F;

  p.maxFlightSpeed = 10.0F;
  p.autoFlightSpeed = 2.0F;

  return p;
}

auto mission::current_waypoint_index() const -> std::optional<std::size_t> {
  std::optional<std::size_t> result;

  if (is_forward_) {
    for (std::size_t i{0}; i < global_waypoints_.size(); ++i) {
      const waypoint& w{global_waypoints_[i]};
      if (result.has_value()) {
        BOOST_VERIFY(!w.is_forward_ready());
        continue;
      }

      if (!w.is_forward_ready()) {
        spdlog::info("Global waypoint #{}", i);
        result = i;
      }
    }
  }
  else {
    for (std::size_t i{global_waypoints_.size()}; i > 0; --i) {
      std::size_t index{i - 1};
      const waypoint& w{global_waypoints_[index]};
      if (result.has_value()) {
        BOOST_VERIFY(!w.is_backward_ready());
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
