#ifndef DRONE_H_
#define DRONE_H_

// clang-format off

/// \mainpage notitle
///
/// \section communication_protocol Communication protocol
///
/// When the onboard service is \ref drone::start "started", it listens for the
/// commands from a user. The figure below shows how the I-Seed Drone Control
/// Android application communicates with the I-Seed onboard service. The
/// Android application runs on an Android phone connected to the drone’s
/// remote control via a USB cable. The commands are noted in uppercase. If
/// extra parameters for a command are needed, they are sent/received as the
/// follow-up message with data.
///
/// \image html protocol.jpg
///
/// When a connection is established between the application and the drone, the
/// application will send the \b PING command and will wait for a reply with the
/// \b PONG command back. This one needed to verify that communication is
/// actually working because the API peculiarity is that all the
/// interconnections return successful codes, but the real packets will start
/// receiving later.
///
/// There is a service running in a background thread on the drone that
/// periodically sends the current mission states and its geographical
/// coordinates. The drone service sends a \b DRONE_INFO command to
/// inform the Android app that a coordinate data and mission state message will
/// follow. The \c mission_path message with the coordinates of the mission
/// path built from the user input polygon will be sent in case of the special
/// drone state called \c PATH_DATA.
///
/// When the drone operator is ready, a mission can be started via the Android
/// application interface. The first step will be to build and approve the
/// mission path by providing an input polygon and sending the \b BUILD_MISSION
/// command. Coordinates will be taken from the Google Map widget of
/// the Android app. In the second step, user sends a \b MISSION_START command
/// to the drone service and actually starts the mission.
/// If the mission path doesn't look appropriate for any reason, the user can
/// clear the mission path by sending \b MISSION_PATH_CANCEL and tweak the
/// input polygon.
///
/// During a mission, a drone operator can pause, resume, or abort its
/// execution. This is achieved by sending \b MISSION_PAUSE,
/// \b MISSION_CONTINUE, and \b MISSION_ABORT commands, respectively,
/// to the drone service.
///
/// Since the effect of applying mission control command is not immediate, and
/// there is a pool of send/received messages, the \c event_id counter is
/// introduced to synchronize the state. After each user's mission command,
/// such as \b MISSION_PAUSE or \b MISSION_CONTINUE, the \c event_id counter
/// increased by one. All the mission states received in DRONE_INFO with the
/// less value in \c event_id will be ignored until the message with updated
/// \c event_id is received.
///
/// The full mission consists of a forward mission (when we take photos and run
/// inference in the background) and a backward mission (when we revisit
/// waypoints where objects were detected and run laser measurements for each
/// object). In both cases, when
/// \ref mission::waypoint_reached "a waypoint is reached", the mission is
/// paused, and custom code is run in the \c action_job thread. If a forward
/// mission is finished, the mission type is
/// \ref mission::set_backward "changed to backward",
/// and a similar cycle continues until ready.
///
/// If an I-Seed is detected in a waypoint during a forward mission, the range
/// between the camera and the I-Seed needs to be known. For this, the DJI
/// Zenmuse H20/T laser rangefinder is used in a waypoint during a backward
/// mission. However, there is no API call for the \c laser_range data
/// available from the Payload SDKs, one needs to use a hack to retrieve this
/// information from the Mobile SDK. The onboard service emits a
/// \b LASER_RANGE_REQUEST
/// command to the Mobile SDK that runs with the Android app to achieve this.
/// After that, the Mobile SDK sends \b LASER_RANGE_RESPONSE with the
/// \c laser_range message back to the
/// onboard service that uses this information to compute the geolocalisation
/// of the I-Seed.
///
/// It's not possible to know beforehand the actual height of the
/// drone above the ground. We only know the actual height after laser
/// measurement of the place received. The \b LASER_RANGE_REQUEST command will
/// be used
/// in this case too. If the actual height is too low or
/// too high, the mission waypoint's height has to be corrected.
///
/// \section control_app I-Seed drone control
///
/// There are a UI system thread that receives UI events and three custom
/// threads: reading/writing from pipe (interconnection) and a polling job that
/// checks the state. The \c executeCommands is a buffer with commands.
///
/// UI thread will process the events from the drone operator.
/// E.g., if a user wants to abort the mission, command \c MISSION_ABORT put
/// into a queue. When \c writePipelineJob takes control of the queue, it reads
/// \c MISSION_ABORT and is responsible for sending it to a drone.
///
/// Read-from-pipe thread with the \c readPipelineJob method will process the
/// commands received from the drone. When the \b PONG command is received, it
/// means communication with a drone is established, and a drone is ready to
/// accept other commands. \b DRONE_INFO command informs that
/// coordinates with the mission state data are prepared. Drone coordinates are
/// used for drawing drone icons in the Google Map widget. Mission state will
/// be used to draw the controlling UI buttons. E.g., if a mission is stopped,
/// UI is switched to the state when an operator can start a new mission.
/// When \c readPipelineJob receives a \b LASER_RANGE_REQUEST request from
/// the drone, it puts \b LASER_RANGE_RESPONSE into a queue. When
/// \c writePipelineJob takes control of the queue, it reads
/// \b LASER_RANGE_RESPONSE and sends it to the drone, but this time command is
/// sent with the data - the value of laser range measurement.
///
/// The \c pollJob method is the main point of calling Mobile SDK API. It is
/// responsible for initialization. It periodically checks the state of a
/// connected smart controller, updates UI widgets, checks the pipeline,
/// checks the GPS signal, enables laser, etc.
///
/// \image html control.jpg
///
/// The following diagram describes the state of the drone control Android
/// application. The initial state is \c WAITING since the drone may already
/// be in the progress of mission execution (e.g., the application is
/// restarted). In the \c READY state, the user can provide a mission input
/// polygon. The cancel button will clear the polygon, and the action button
/// will send the \b BUILD_MISSION command.
///
/// When the mission path is ready, the special state \c PATH_DATA will be
/// received from the onboard service, and the \c mission_path message will be
/// the next packet. After this state, the next state will be \c PATH. In the
/// \c PATH state user can cancel the mission by sending \b MISSION_PATH_CANCEL
/// or start the mission by sending \b MISSION_START. State \c EXECUTING is a
/// normal state when a mission is in progress. State \c PAUSED received when
/// a user pauses the mission. In both those states mission can be aborted by
/// sending \b MISSION_ABORT.
///
/// \image html states_control.jpg
///
/// \note Java, Android
///
/// \section onboard_service_threads I-Seed onboard service
///
/// \anchor thread_psdk_callback
/// \anchor thread_inference
/// \anchor thread_receive_data
/// \anchor thread_send_data
/// \anchor thread_action
///
/// After the drone is started, there are five threads:
/// - Payload SDK callback thread, that's where asynchronous callbacks from
///   Payload SDK live. E.g., we can receive an event that the mission is
///   finished or the waypoint reached (see \ref mission and
///   \ref mission_state). Note that there should be no time-consuming calls
///   or blocks here. The callback should be finished as soon as possible
/// - \c inference_job checks for
///   \ref camera::check_sdcard "new files on SDCard", and if a new JPG
///   file is found, it
///   \ref inference::run "launches the inference"
///   and \ref mission::save_detection "saves the result back"
///   to the \ref waypoint::save_detection "waypoint data"
/// - \c receive_data_job is reading pipe. That's where commands and data (like
///   \ref laser_range::value_received) from the Android app come from
/// - \c send_data_job is writing commands to pipe, commands are read from
///   \c execute_commands_ queue, if needed extra data send as a follow-up
/// - \c action_job is executed when we reach a waypoint. Most of the mission
///   it's in the waiting state. That's where we can
///   \ref camera::shoot_photo "start shooting a photo",
///   \ref laser_range::latest "request for laser range", and calculate the
///   \ref converter "ECEF coordinates" of
///   \ref detection_result "detected objects"
///
/// \image html service.jpg
///
/// The following diagram describes the state of the drone onboard service.
/// The initial state is \c ready. In this state, the service emits \c READY
/// for drone control until the user receives the input polygon, and the
/// service \ref mission_builder "builds the mission path".
/// When the mission path is ready, the state
/// is switched to a special state, \c path_data, which emits \c PATH_DATA and
/// a message \c mission_path. Immediately after \c PATH_DATA is sent, the
/// state is switched to the \c path state, emitting \c PATH. In this state,
/// the user can cancel the mission or start it. Starting the mission is a
/// two-step process. The first \c init step will fetch the mission path,
/// switching to \c forward_wait_start, and the second \c start step will
/// actually start the mission, switching to \c forward_wait_update.
///
/// In all the states except \c forward_wait_update, \c backward_wait_update,
/// \c forward_executing, \c backward_executing, the update callbacks received
/// from Payload SDK will be ignored. The
/// \c forward_wait_update/\c backward_wait_update
/// will be switched to
/// \c forward_executing/\c backward_executing
/// once the Payload SDK update is received. In these
/// states \c EXECUTING/PAUSED emitted. All other states that were not
/// mentioned so far will emit \c WAITING. In
/// \c forward_executing/backward_executing states, the method \c abort can be
/// called if a user
/// wants to abort the mission (the following method will be \c stop), or the
/// last fake waypoint reached, or the same mission but with the tweaked height
/// need to be restarted (forward mission only, following method will be
/// \c start). When the finish event is received from the Payload SDK
/// callback, the state is switched to \c forward_finished/backward_finished.
/// \c set_backward will switch the mission to \c backward_wait_start for the
/// forward mission method. Another way to reach \c backward_wait_start is from
/// \c forward_wait_start (when the last fake waypoint is reached and we want
/// to start the backward mission). If no objects are detected in the forward
/// mission, the method \c stop will move \c backward_wait_start to ready.
///
/// \image html mission_state.jpg
///
/// \note C++, Linux

// clang-format on

#include <signal.h>  // sig_atomic_t

#include <atomic>
#include <cstdint>  // uint16_t

#include "camera.h"
#include "condition_flag.h"
#include "home_altitude.h"
#include "laser_range.h"
#include "mission.h"
#include "simulator.h"

using T_DjiMopChannelHandle = void*;

/// \brief Stop the process because of interruption event: exception or SIGINT
class job_interrupted_event : public std::exception {};

/// \brief High level drone control
class drone {
 public:
  /// \brief Create drone object
  drone();
  ~drone();

  /// \cond private
  drone(const drone&) = delete;
  drone(drone&&) = delete;

  drone& operator=(const drone&) = delete;
  drone& operator=(drone&&) = delete;
  /// \endcond

  /// \brief Start the drone work
  void start();

  static constexpr int32_t protocol_version{
      16};  // Keep it consistent with Mobile SDK (see 'protocolVersion')

  static constexpr E_DjiMountPosition m_pos{
      DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1};

 private:
  template <class T>
  static const T& cast_dji(const uint8_t* data, uint16_t data_size);

  static T_DjiReturnCode quaternion_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode rc_callback(const uint8_t* data, uint16_t data_size,
                                     const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode position_fused_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
#if !defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  // RTK: ON
  static T_DjiReturnCode rtk_position_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);
#endif

  static T_DjiReturnCode gimbal_callback(const uint8_t* data,
                                         uint16_t data_size,
                                         const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode mission_event_callback(
      T_DjiWaypointV2MissionEventPush event_data);
  static T_DjiReturnCode mission_state_callback(
      T_DjiWaypointV2MissionStatePush state_data);
  static T_DjiReturnCode homepoint_callback(
      const uint8_t* data, uint16_t data_size,
      const T_DjiDataTimestamp* timestamp);

  static constexpr float invalid_homepoint_altitude_{
      std::numeric_limits<float>::lowest()};

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/module/fc-subscription.html#typedef-struct-t-djifcsubscriptionquaternion
  // Read from 'T_DjiFcSubscriptionQuaternion':
  // - dji_f32_t q0
  // - dji_f32_t q1
  // - dji_f32_t q2
  // - dji_f32_t q3
  static std::atomic<float> drone_yaw_;
  static std::atomic<float> drone_pitch_;
  static std::atomic<float> drone_roll_;

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/module/fc-subscription.html#typedef-struct-t-djifcsubscriptionpositionfused
  // Read from 'T_DjiFcSubscriptionPositionFused':
  // - dji_f64_t longitude
  // - dji_f64_t latitude
  // - dji_f32_t altitude
  static std::atomic<double> drone_longitude_;
  static std::atomic<double> drone_latitude_;
  static std::atomic<float> drone_altitude_;

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/module/fc-subscription.html#34
  // Read from 'GimbalSingleData':
  // - dji_f32_t pitch
  // - dji_f32_t roll
  // - dji_f32_t yaw
  static std::atomic<float> gimbal_yaw_;
  static std::atomic<float> gimbal_pitch_;
  static std::atomic<float> gimbal_roll_;

  // https://developer.dji.com/doc/payload-sdk-api-reference/en/module/fc-subscription.html#definition-enum-and-struct
  // Read from 'T_DjiFcSubscriptionAltitudeOfHomePoint':
  // - dji_f32_t altitude
  static std::atomic<float> homepoint_altitude_;

  static std::atomic<int16_t> rc_mode_;
  static std::mutex execute_commands_mutex_;
  static std::list<interconnection::command_type::command_t> execute_commands_;

  std::mutex action_job_mutex_;

  bool interrupt_condition() const;

  void action_job();
  void action_job_internal();

  static void align_gimbal();
  static attitude rotate_gimbal(float x, float y, double drone_heading_degree);

  void inference_job();

  void receive_data_job();
  void receive_data_job_internal();

  void send_data_job();
  void send_data_job_internal();

  void send_command(interconnection::command_type::command_t);
  void receive_data(std::string* buffer);
  uint32_t receive_next_packet_size();
  void send_data(std::string& buffer);
  void send_next_packet_size(uint32_t size);
  int32_t receive_event_id();
  interconnection::drone_info::state_t send_drone_info();

  /// \anchor drone's next_mission
  void next_mission();

  float latest_laser_range();

  static constexpr uint16_t channel_id{
      9745};  // Just a random number. Keep it consistent with Mobile SDK
  static constexpr int pkg_index{0};
  static constexpr int timeout{20};

  static mission mission_;
  T_DjiMopChannelHandle channel_handle_{nullptr};
  camera camera_;

  uint32_t packet_size_{0};

  std::atomic<bool> connection_closed_{false};
  std::atomic<bool> exception_caught_{false};

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  static simulator simulator_;
#else
  laser_range laser_range_;
#endif

  static condition_flag action_flag_;

  static void sigint_handler(int);
  static std::atomic<bool> sigint_received_;
  home_altitude home_altitude_;
};

#endif  //  DRONE_H_
