#include "mission.h"

#include "utils.h" // deg2rad

void mission::init(double lat, double lon) {
  {
    std::lock_guard<std::mutex> lock(m_);

    spdlog::info("Mission parameters: lat({}), lon({})", lat, lon);

    // FIXME (points from polygons)
    // FIXME (action at waypoint?)
    global_waypoints_.clear();
    global_waypoints_.emplace_back(lat, lon + 0.0001);
    global_waypoints_.emplace_back(lat, lon + 0.0002);
    // FIXME (remove) global_waypoints_.emplace_back(lat, lon + 0.0003);
    // FIXME (remove) global_waypoints_.emplace_back(lat, lon + 0.0004);
  }

  upload_mission_and_start();
}

auto mission::waypoint_reached(double laser_range, double* waypoint_lat, double* waypoint_lon, double* waypoint_alt) -> waypoint_action {
  std::lock_guard<std::mutex> lock(m_);

  waypoint* ptr{current_waypoint()};

  if (ptr == nullptr) {
    return waypoint_action::abort;
  }

  waypoint& w{*ptr};

  if (std::abs(laser_range - waypoint::expected_height) > 1.0) {
    spdlog::info("Bad laser range, waypoint altitude tweak");
    BOOST_VERIFY(w.is_default_altitude());
    w.set_custom_altitude(laser_range);
    BOOST_VERIFY(!w.is_default_altitude());
    return waypoint_action::restart;
  }

  w.set_ready();

  BOOST_VERIFY(waypoint_lat != nullptr);
  *waypoint_lat = w.lat();

  BOOST_VERIFY(waypoint_lon != nullptr);
  *waypoint_lon = w.lon();

  BOOST_VERIFY(waypoint_alt != nullptr);
  *waypoint_alt = w.altitude();

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
  for (const waypoint& w : global_waypoints_) {
    if (w.is_ready()) {
      BOOST_VERIFY(waypoints_.empty());
      continue;
    }
    waypoints_.push_back(make_waypoint(w.lat(), w.lon(), w.altitude()));
  }
  BOOST_VERIFY(!waypoints_.empty());
  if (waypoints_.size() == 1) {
    // Duplicate the last and ignore it when reached
    // Tweak the height to avoid "points are too close" error
    const waypoint& w{global_waypoints_.back()};
    BOOST_VERIFY(!w.is_ready());
    waypoints_.push_back(make_waypoint(w.lat(), w.lon(), w.altitude() + 5.0));
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

T_DjiWaypointV2 mission::make_waypoint(double latitude, double longitude, double relative_height) {
  spdlog::info("Add waypoint lat({}), lon({}), height({})", latitude, longitude, relative_height);
  T_DjiWaypointV2 p;

  p.latitude = latitude * deg2rad;
  p.longitude = longitude * deg2rad;
  p.relativeHeight = relative_height;

  p.waypointType = DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;

  // Aircraft's heading will always be in the direction of flight
  p.headingMode = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;

  // FIXME (use for the backward mission)
  // p.headingMode = DJI_WAYPOINT_V2_HEADING_WAYPOINT_CUSTOM;

  p.config.useLocalCruiseVel = 0;  // set local waypoint's cruise speed
  p.config.useLocalMaxVel = 0;     // set local waypoint's max speed

  p.dampingDistance = 40;  // cm
  p.heading = 0.0; // FIXME: use for DJI_WAYPOINT_V2_HEADING_WAYPOINT_CUSTOM

  p.turnMode = DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

  // unused
  p.pointOfInterest.positionX = 0.0F;
  p.pointOfInterest.positionY = 0.0F;
  p.pointOfInterest.positionZ = 0.0F;

  p.maxFlightSpeed = 10.0F;
  p.autoFlightSpeed = 2.0F;

  return p;
}

auto mission::current_waypoint() -> waypoint* {
  waypoint* result{nullptr};
  for (std::size_t i{0}; i < global_waypoints_.size(); ++i) {
    waypoint& w{global_waypoints_[i]};
    if (result != nullptr) {
      BOOST_VERIFY(!w.is_ready());
      continue;
    }

    if (!w.is_ready()) {
      spdlog::info("Global waypoint #{}", i);
      result = &w;
    }
  }
  return result;
}
