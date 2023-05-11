#ifndef DRONE_H_
#define DRONE_H_

// clang-format off

/// \mainpage notitle
///
/// \section communication_protocol Communication protocol
///
/// When the onboard service is \ref drone::start "started", it listens for the
/// commands from a user. Commands are sent from the Android control
/// application. If extra parameters for a command are needed, they are
/// sent/received as the follow-up message with data.
///
/// \image html protocol.jpg
///
/// \note There is no public API in Payload SDK (onboard service) to get the
///   laser measurement value, so we have to ask Mobile SDK (drone control) and
///   send the value back to the drone
///
/// \section control_app I-Seed drone control
///
/// There are a UI system thread that receives UI events and three custom
/// threads: reading/writing from pipe (interconnection) and a polling job that
/// checks the state. The \c executeCommands is a buffer with commands.
///
/// E.g., if a user wants to abort the mission, command \c MISSION_ABORT put
/// into a queue. When \c writePipelineJob takes control of the queue, it reads
/// \c MISSION_ABORT and is responsible for sending it to a drone.
///
/// Or when \c readPipelineJob receives a \c LASER_RANGE request from the drone,
/// it is put into a queue. When \c writePipelineJob takes control of the queue,
/// it reads \c LASER_RANGE and sends it to the drone, but this time command is
/// sent with the data - the value of laser range measurement.
///
/// \image html control.jpg
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
///   \ref camera_psdk::check_sdcard "new files on SDCard", and if a new JPG
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
///   \ref camera_psdk::shoot_photo "start shooting a photo",
///   \ref laser_range::latest "request for laser range", and calculate the
///   \ref converter "ECEF coordinates" of
///   \ref detection_result "detected objects"
///
/// \image html service.jpg
///
/// \note C++, Linux

// clang-format on

#include <signal.h>  // sig_atomic_t

#include <atomic>
#include <cstdint>  // uint16_t

#include "camera_psdk.h"
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
      11};  // Keep it consistent with Mobile SDK (see 'protocolVersion')

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
  void send_data(std::string& buffer);

  /// \anchor drone's next_mission
  void next_mission();

  static constexpr uint16_t channel_id{
      9745};  // Just a random number. Keep it consistent with Mobile SDK
  static constexpr int pkg_index{0};
  static constexpr int timeout{20};

  static mission mission_;
  T_DjiMopChannelHandle channel_handle_{nullptr};
  camera_psdk camera_psdk_;

  uint32_t command_bytes_size_{0};
  uint32_t pin_coordinates_bytes_size_{0};
  uint32_t laser_range_bytes_size_{0};
  std::atomic<bool> connection_closed_{false};
  std::atomic<bool> exception_caught_{false};

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  static simulator simulator_;
#endif

  static condition_flag action_flag_;

  static void sigint_handler(int);
  static std::atomic<bool> sigint_received_;
  home_altitude home_altitude_;

  laser_range laser_range_;
};

#endif  //  DRONE_H_
