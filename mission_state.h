#ifndef MISSION_STATE_H_
#define MISSION_STATE_H_

#include <mutex>
#include <set>

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
  uint8_t state_{0};
  uint16_t waypoint_index_{0};

  // https://github.com/dji-sdk/Payload-SDK/blob/3.3/psdk_lib/include/dji_waypoint_v2_type.h#L1055-L1070
  enum {
    ground_station_not_start = 0x0,
    mission_prepared = 0x1,
    enter_mission = 0x2,
    execute_flying_route_mission = 0x3,
    pause_state = 0x4,
    enter_mission_after_ending_pause = 0x5,
    exit_mission = 0x6
  };

  std::set<uint16_t> already_executed_;

  mutable std::mutex m_;
};

#endif  // MISSION_STATE_H_
