#ifndef MISSION_STATE_H_
#define MISSION_STATE_H_

#include <dji_waypoint_v2.h>  // T_DjiWaypointV2MissionEventPush

#include <mutex>
#include <set>

#include "interconnection.pb.h"

/// \brief Class to watch the global mission state that will be sent over
///   interconnection
/// \details Since it's thread-safe and used in Payload SDK callbacks,
///   all the methods should return flow control quickly, avoid long wait.
///   Any Payload SDK API related to WaypointV2 mission should
///   be called in \ref mission. Waypoints information should be saved
///   in \ref mission too. Argument \b event_id indicates a user command.
class mission_state {
 public:
  static constexpr int32_t internal_event_id{-1};

  /// \brief The mission path is constructed and ready to be sent to a user
  ///   for confirmation
  /// \details \b ready -> \b path_data
  void mission_path_ready(int32_t event_id);

  /// \brief A user request to clear the current mission path. Either the path
  ///   is invalid, or the user wants to change it
  /// \details \b path_data -> \b ready
  void mission_path_cancel(int32_t event_id);

  /// \brief The mission path is available in current state
  bool mission_path_available() const;

  /// \brief Initial fetch of the path before running forward/backward missions
  /// \details \b path -> \b forward_wait_start
  /// \note Update events still ignored
  void init();

  /// \brief Start mission (either forward or backward). Update events can be
  /// processed
  /// \details \b forward_wait_start -> \b forward_wait_update
  /// \details \b backward_wait_start -> \b backward_wait_update
  void start(int32_t event_id);

  /// \brief Pause mission
  /// \details \b forward_executing or \b backward_executing
  void pause(int32_t event_id);

  /// \brief Stop the global mission execution
  /// \details \b forward_wait_start -> \b ready (abort execution)
  /// \details \b backward_wait_start -> \b ready (no objects detected -
  ///   no backward mission, aborted execution)
  /// \details \b backward_finished -> \b ready (success)
  void stop();

  /// \brief Abort mission. Ignore update events
  /// \details \b backward_executing -> \b backward_wait_start
  /// \details \b forward_executing -> \b forward_wait_start
  /// \note
  ///     - forward mission restart on height tweak
  ///     - mission abort on last fake waypoint
  ///     - user mission abort
  void abort(int32_t event_id);

  /// \brief Resume mission
  /// \details \b forward_executing or \b backward_executing
  void resume(int32_t event_id);

  /// \brief Update mission \b event_id
  void update_event_id(int32_t event_id);

  /// \brief Verify state correctness when waypoint reached
  /// \details \b forward_executing or \b backward_executing
  void waypoint_reached() const;

  /// \brief Forward mission is in progress
  bool is_forward() const;

  /// \brief Switch to backward mission
  /// \details \b forward_wait_start -> \b backward_wait_start
  ///     (last fake waypoint)
  /// \details \b forward_finished -> \b backward_wait_start
  ///     (forward success)
  void set_backward();

  /// \brief Check if forward/backward mission finished
  bool is_finished() const;

  /// \brief Check if mission is ready to start
  bool is_ready() const;

  /// \brief Check if global mission is paused
  bool is_paused() const;

  /// \brief Check if state accepts user's command pause/resume/abort
  bool user_cmd_accepted() const;

  /// \brief Process mission event received from Payload SDK
  /// \return notify_finished
  bool update(T_DjiWaypointV2MissionEventPush event_data);

  /// \brief Process state update event received from Payload SDK
  /// \return auto [mission_started, notify]
  std::pair<bool, bool> update(T_DjiWaypointV2MissionStatePush state_data);

  /// \brief Current drone's state to sent to the drone control Android
  ///     application
  /// \return auto [event_id, state]
  /// \note Can change the global state \b path_data -> \b path
  std::pair<int32_t, interconnection::drone_info::state_t> get_state();

 private:
  std::pair<int32_t, interconnection::drone_info::state_t> get_state(
      interconnection::drone_info::state_t new_state);
  const char* state_name() const;
  bool check_updates() const;
  void set_next_state(interconnection::drone_info::state_t, int32_t event_id);

  enum global_state {
    ready,
    path_data,
    path,
    forward_wait_start,
    forward_wait_update,
    forward_executing,
    forward_finished,
    backward_wait_start,
    backward_wait_update,
    backward_executing,
    backward_finished
  };

  global_state global_state_{ready};

  // Can be read only in 'forward_executing' or 'backward_executing' state
  uint8_t state_{0};
  uint16_t waypoint_index_{0};

  // This one is global. Don't change even if connection is restarted
  int32_t event_id_{0};

  std::optional<interconnection::drone_info::state_t> next_state_;
  int32_t next_event_id_{0};

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
