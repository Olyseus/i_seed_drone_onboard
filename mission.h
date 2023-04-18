#ifndef MISSION_H_
#define MISSION_H_

#include <dji_waypoint_v2.h>  // T_DjiWaypointV2

#include <mutex>
#include <vector>

#include "interconnection.pb.h"
#include "mission_state.h"
#include "waypoint.h"

class home_altitude;

enum class waypoint_action { ok, abort, restart };

class mission {
 public:
  mission() noexcept;
  ~mission();

  mission(const mission&) = delete;
  mission(mission&&) = delete;
  mission& operator=(const mission&) = delete;
  mission& operator=(mission&&) = delete;

  // Apply user coordinates received from 'MISSION_START'
  // thread: receive data
  bool init(double lat, double lon);

  // - Upload initial mission (thread: receive data)
  // - Upload forward mission with tweaked altitude (thread: action)
  // - Upload backward mission, after 'set_backward' (thread: action)
  // @return false if no waypoints left to visit
  bool upload_mission_and_start();

  // thread: action
  std::pair<waypoint_action, std::size_t> waypoint_reached(float laser_range);

  // thread: action
  waypoint get_waypoint_copy(std::size_t index) const;

  // thread: inference
  void save_detection(std::size_t index, const detection_result&);

  // thread: action (called from 'drone::next_mission')
  void set_backward();

  // thread: action
  bool is_forward() const;

  // first flag to check after the action thread notified
  // thread: action
  bool is_finishing() const;

  // thread: PSDK callback
  // @return true if action thread need to be notified
  bool update(T_DjiWaypointV2MissionEventPush event_data);

  // thread: PSDK callback
  // auto [mission_started, notify]
  std::pair<bool, bool> update(T_DjiWaypointV2MissionStatePush state_data);

  // - Abort mission on user request (thread: receive data)
  // - Abort mission on fake waypoint,
  //   start backward mission if needed (thread: action)
  // - Abort mission when altitude need to be tweaked,
  //   only forward (thread: action)
  void abort_mission();

  // - Abort mission on user request (thread: receive data)
  // - Backward mission finished succesfully (thread: action)
  // - Forward mission finished, but backward mission not started because no
  //   object detected (thread: action)
  void mission_stop(home_altitude& h);

  // When 'MISSION_CONTINUE' received from user
  // thread: receive data
  bool resume();

  // Send current state periodically
  // thread: send data
  interconnection::drone_coordinates::state_t get_state() const;

 private:
  T_DjiWaypointV2 make_waypoint(const waypoint& w) const;
  std::optional<std::size_t> current_waypoint_index() const;

  mutable std::mutex m_;

  std::vector<T_DjiWaypointV2> waypoints_;
  std::vector<waypoint> global_waypoints_;

  bool in_progress_{false};
  bool is_forward_{false};

  mutable std::mutex is_finishing_mutex_;
  bool is_finishing_{false};

  mission_state mission_state_;
};

#endif  // MISSION_H_
