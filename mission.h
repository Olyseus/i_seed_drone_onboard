#ifndef MISSION_H_
#define MISSION_H_

#include <mutex>
#include <vector>

#include <dji_waypoint_v2.h> // T_DjiWaypointV2

#include "waypoint.h"

class mission_state;

enum class waypoint_action {
  ok,
  abort,
  restart
};

class mission {
 public:
  explicit mission(mission_state&);
  ~mission();

  mission(const mission&) = delete;
  mission(mission&&) = delete;
  mission& operator=(const mission&) = delete;
  mission& operator=(mission&&) = delete;

  void init(double lat, double lon);
  waypoint_action waypoint_reached(double laser_range, std::size_t* waypoint_index);
  void upload_mission_and_start();

  waypoint get_waypoint_copy(std::size_t index) const;
  void save_detection(std::size_t index, const detection_result&);

 private:
  T_DjiWaypointV2 make_waypoint(double latitude, double longitude, double relative_height);
  std::optional<std::size_t> current_waypoint_index() const;

  mutable std::mutex m_;

  std::vector<T_DjiWaypointV2> waypoints_;
  std::vector<waypoint> global_waypoints_;

  mission_state& mission_state_;
};

#endif // MISSION_H_
