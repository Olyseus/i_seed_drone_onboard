#ifndef MISSION_STATE_H_
#define MISSION_STATE_H_

#include <mutex>

#include <dji_waypoint_v2.h> // T_DjiWaypointV2MissionEventPush

class mission_state {
 public:
  static constexpr uint16_t invalid_waypoint{std::numeric_limits<uint16_t>::max()};

  void start();
  void finish();

  bool is_started() const;

  void update(T_DjiWaypointV2MissionEventPush event_data);
  uint16_t update(T_DjiWaypointV2MissionStatePush state_data);

 private:
  const char* state_name() const;

  bool is_started_{false};
  bool initial_update_received_{false};
  int64_t state_{0};
  uint16_t waypoint_index_{0};

  // Unknown state, receiving while using simulator,
  // after receiving disconnected multiple times
  static constexpr int64_t mission_unknown_state_7_{7};

  mutable std::mutex m_;
};

#endif  // MISSION_STATE_H_
