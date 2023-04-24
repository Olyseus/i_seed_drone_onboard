#ifndef MISSION_STATE_H_
#define MISSION_STATE_H_

#include <dji_waypoint_v2.h>  // T_DjiWaypointV2MissionEventPush

#include <mutex>
#include <set>

#include "interconnection.pb.h"

/// \brief Class to watch the mission state that will be sent over
///   interconnection
class mission_state {
 public:
  /// \brief Start mission. Update events can be processed
  void start();

  /// \brief Finish mission. Ignore update events
  void finish();

  /// \brief Mission is started
  bool is_started() const;

  /// \brief Process mission event received from Payload SDK
  /// \return notify_finished
  bool update(T_DjiWaypointV2MissionEventPush event_data);

  /// \brief Process state update event received from Payload SDK
  /// \return auto [mission_started, notify, notify_finished]
  std::tuple<bool, bool, bool> update(
      T_DjiWaypointV2MissionStatePush state_data);

  /// \brief Current drone's state to sent to the drone control Android
  ///     application
  interconnection::drone_coordinates::state_t get_state() const;

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
    exit_mission = 0x6,
    end_of_waypoint_mission =
        0x7  // https://sdk-forum.dji.net/hc/en-us/requests/76600
  };

  std::set<uint16_t> already_executed_;

  mutable std::mutex m_;
};

#endif  // MISSION_STATE_H_
