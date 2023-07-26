#ifndef MISSION_H_
#define MISSION_H_

#include "lat_lon.h"
#include "mission_state.h"
#include "waypoint.h"

class home_altitude;

/// \brief Action to take when waypoint reached
enum class waypoint_action {
  /// \brief Process waypoint
  ok,

  /// \brief Abort mission on last fake waypoint
  /// \details The minimum number of waypoints in a mission is two. So when we
  /// need to start a mission with one waypoint, we must add a fake one to the
  /// list. When such a waypoint is reached, we immediately abort the mission.
  abort,

  /// \brief Tweak altitude and restart mission
  /// \details It's not possible to know beforehand the actual height of the
  /// drone above the ground. We only know the actual height after laser
  /// measurement of the place received. If the actual height is too low or
  /// too high, the mission waypoint's height has to be corrected. After that,
  /// the mission is restarted.
  restart
};

/// \brief Drone's mission control
/// \note Class is thread-safe
class mission {
 public:
  mission() noexcept;
  ~mission();

  /// \cond private
  mission(const mission&) = delete;
  mission(mission&&) = delete;
  mission& operator=(const mission&) = delete;
  mission& operator=(mission&&) = delete;
  /// \endcond

  /// \brief The mission path is ready. It is constructed based on an input
  ///   polygon
  void mission_path_ready(std::vector<lat_lon> mission_path, int32_t event_id);

  /// \brief A user request to clear the current mission path
  void mission_path_cancel(int32_t event_id);

  /// \brief Get the costructred mission path
  const std::vector<lat_lon> get_mission_path() const;

  /// \brief 'MISSION_START' command received from user
  /// \note \ref thread_receive_data "Thread: receive data"
  void init();

  /// \brief Upload and start mission
  /// \details
  ///   - Upload initial mission
  ///     (\ref thread_receive_data "Thread: receive data")
  ///   - Upload forward mission with tweaked altitude
  ///     (\ref thread_action "Thread: action")
  ///   - Upload backward mission, after \ref set_backward
  ///     (\ref thread_action "Thread: action")
  /// \return \b false if no waypoints left to visit
  bool upload_mission_and_start(int32_t event_id);

  /// \brief Ask for an action when waypoint reached
  /// \param[in] laser_range Use the laser range to determine the real drone
  ///     height
  /// \return [waypoint_action, waypoint index]
  /// \note \ref thread_action "Thread: action"
  std::pair<waypoint_action, std::size_t> waypoint_reached(float laser_range);

  /// \brief Waypoint copy by index
  /// \return \ref waypoint
  /// \note \ref thread_action "Thread: action"
  waypoint get_waypoint_copy(std::size_t index) const;

  /// \brief Save the detected objects to waypoint data
  /// \param[in] index Waypoint index
  /// \param[in] result Detection result
  /// \note \ref thread_inference "Thread: inference"
  void save_detection(std::size_t index, const detection_result& result);

  /// \brief The forward mission is finished,
  ///   now set the current mission status to backward
  /// \note \ref thread_action "Thread: action" (called from
  ///   \ref drone \::next_mission)
  void set_backward();

  /// \brief Check if mission is forward
  /// \note \ref thread_action "Thread: action"
  bool is_forward() const;

  /// \brief Flag to indicate that mission is finished
  /// \note This is a first flag that is checked when then action thread
  ///     is notified
  /// \note \ref thread_action "Thread: action"
  bool is_finished() const;

  /// \brief Flag to indicate that mission is ready to be started
  bool is_ready() const;

  /// \brief Flag to indicate that execution of mission is paused
  bool is_paused() const;

  /// \brief Flag to indicate that mission is in the stable state and is
  ///   ready to accept the user commands pause/resume/abort
  bool user_cmd_accepted() const;

  /// \brief Update mission \b event_id
  /// \details Apply new value without affecting the state because the state
  ///   is no longer actual. E.g., trying to abort a mission that is already
  ///   finished.
  void update_event_id(int32_t event_id);

  /// \brief Process mission event received from Payload SDK
  /// \note \ref thread_psdk_callback "Thread: Payload SDK callback"
  /// \return \b true if action thread need to be notified
  bool update(T_DjiWaypointV2MissionEventPush event_data);

  /// \brief Process state update event received from Payload SDK
  /// \note \ref thread_psdk_callback "Thread: Payload SDK callback"
  /// \return auto [mission_started, notify]
  std::pair<bool, bool> update(T_DjiWaypointV2MissionStatePush state_data);

  /// \brief Abort the mission
  /// \details
  ///   - Abort mission on user request
  ///     (\ref thread_user_control "Thread: user control")
  ///   - Abort mission on fake waypoint,
  ///     start backward mission if needed
  ///     (\ref thread_action "Thread: action")
  ///   - Abort mission when altitude need to be tweaked,
  ///     only forward (\ref thread_action "Thread: action")
  void abort(int32_t event_id);

  /// \brief Pause the mission
  void pause(int32_t event_id);

  /// \brief Stop the mission
  /// \details
  ///   - Abort mission on user request
  ///     (\ref thread_user_control "Thread: user control")
  ///   - Backward mission finished succesfully
  ///     (\ref thread_action "Thread: action")
  ///   - The forward mission finished, but the backward mission not started
  ///     because no objects were detected
  ///     (\ref thread_action "Thread: action")
  void stop(home_altitude& h);

  /// \brief Resume the mission
  /// \details Triggered when \c MISSION_CONTINUE received from user
  /// \note \ref thread_receive_data "Thread: receive data"
  void resume(int32_t event_id);

  /// \brief Periodically send the current state to the drone control Android
  ///     application
  /// \note \ref thread_send_data "Thread: send data"
  /// \return auto [event_id, state]
  std::pair<int32_t, interconnection::drone_info::state_t> get_state();

 private:
  T_DjiWaypointV2 make_waypoint(const waypoint& w, bool is_forward) const;
  std::optional<std::size_t> current_waypoint_index(bool is_forward) const;

  // Mutex for waypoints, 'mission_state_' is thread-safe
  mutable std::mutex m_;

  std::vector<lat_lon> mission_path_;

  std::vector<T_DjiWaypointV2> waypoints_;
  std::vector<waypoint> global_waypoints_;

  mission_state mission_state_;
};

#endif  // MISSION_H_
