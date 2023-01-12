#ifndef DRONE_H_
#define DRONE_H_

#include <atomic>
#include <cmath>  // M_PI
#include <condition_variable>
#include <cstdint>  // uint16_t
#include <list>
#include <mutex>
#include <signal.h>        // sig_atomic_t

#include <dji_waypoint_v2.h> // T_DjiWaypointV2

#include "camera_psdk.h"
#include "interconnection.pb.h"
#include "mission_state.h"
#include "simulator.h"

using T_DjiMopChannelHandle = void*;

class job_interrupted_event {};

class drone {
 public:
  drone();
  ~drone();

  drone(const drone&) = delete;
  drone(drone&&) = delete;

  drone& operator=(const drone&) = delete;
  drone& operator=(drone&&) = delete;

  void start();

  static constexpr int32_t protocol_version{
      6};  // Keep it consistent with Mobile SDK

  static constexpr E_DjiMountPosition m_pos{DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1};

 private:
  static T_DjiReturnCode quaternion_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode rc_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode position_fused_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode gimbal_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode mission_event_callback(T_DjiWaypointV2MissionEventPush event_data);
  static T_DjiReturnCode mission_state_callback(T_DjiWaypointV2MissionStatePush state_data);

  static std::atomic<double> drone_yaw_;
  static std::atomic<double> drone_pitch_;
  static std::atomic<double> drone_roll_;
  static std::atomic<double> drone_longitude_;
  static std::atomic<double> drone_latitude_;
  static std::atomic<double> drone_altitude_;
  static std::atomic<int16_t> rc_mode_;
  static mission_state mission_state_;
  static std::mutex execute_commands_mutex_;
  static std::list<interconnection::command_type::command_t> execute_commands_;

  bool interrupt_condition() const;

  void action_job();
  void action_job_internal();

  void gimbal_job();
  void gimbal_job_internal();

  void inference_job();

  void receive_data_job();
  void receive_data_job_internal();

  void send_data_job();
  void send_data_job_internal();

  void send_command(interconnection::command_type::command_t);
  void receive_data(std::string* buffer);
  void send_data(std::string& buffer);
  T_DjiWaypointV2 make_waypoint(double latitude, double longitude,
                                      float relative_height);

  static constexpr uint16_t channel_id{
      9745};  // Just a random number. Keep it consistent with Mobile SDK
  static constexpr int pkg_index{0};
  static constexpr int timeout{20};

  static constexpr double pi_degree{180.0};
  static constexpr double rad2deg{pi_degree / M_PI};
  static constexpr double deg2rad{M_PI / pi_degree};

  T_DjiMopChannelHandle channel_handle_{nullptr};
  camera_psdk camera_psdk_;

  uint32_t command_bytes_size_{0};
  uint32_t pin_coordinates_bytes_size_{0};
  std::atomic<bool> connection_closed_{false};
  std::atomic<bool> exception_caught_{false};

  std::vector<T_DjiWaypointV2> waypoints_;

#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  static simulator simulator_;
#endif

  static std::mutex action_mutex_;
  static std::condition_variable action_condition_variable_;
  static uint16_t action_waypoint_;

  static void sigint_handler(int);
  static std::atomic<bool> sigint_received_;
};

#endif  //  DRONE_H_
