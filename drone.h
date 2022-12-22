#ifndef DRONE_H_
#define DRONE_H_

#include <signal.h>        // sig_atomic_t

#include <atomic>
#include <cstdint>  // uint16_t
#include <list>
#include <mutex>

#include <dji_waypoint_v2.h> // T_DjiWaypointV2

#include "interconnection.pb.h"
#include "mission_state.h"

using T_DjiMopChannelHandle = void*;

class drone {
 public:
  drone();
  ~drone();

  drone(const drone&) = delete;
  drone(drone&&) = delete;

  drone& operator=(const drone&) = delete;
  drone& operator=(drone&&) = delete;

  void start();

 private:
  static T_DjiReturnCode quaternion_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode rc_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode position_fused_callback(const uint8_t* data, uint16_t data_size, const T_DjiDataTimestamp* timestamp);
  static T_DjiReturnCode mission_event_callback(T_DjiWaypointV2MissionEventPush event_data);
  static T_DjiReturnCode mission_state_callback(T_DjiWaypointV2MissionStatePush state_data);

  static double drone_yaw_;
  static double drone_longitude_;
  static double drone_latitude_;
  static int16_t rc_mode_;
  static mission_state mission_state_;
  static std::mutex m_;
  static std::list<interconnection::command_type::command_t> execute_commands_;

  void receive_data_job();
  void send_data_job();
  void send_command(interconnection::command_type::command_t);
  void receive_data(std::string* buffer);
  void send_data(std::string& buffer);
  T_DjiWaypointV2 make_waypoint(double latitude, double longitude,
                                      float relative_height);

  static constexpr uint16_t channel_id{
      9745};  // Just a random number. Keep it consistent with Mobile SDK
  static constexpr int32_t protocol_version{
      6};  // Keep it consistent with Mobile SDK
  static constexpr int pkg_index{0};
  static constexpr int timeout{10};

  static constexpr double pi_degree{180.0};
  static constexpr double rad2deg{pi_degree / M_PI};
  static constexpr double deg2rad{M_PI / pi_degree};

  T_DjiMopChannelHandle channel_handle_{nullptr};

  uint32_t command_bytes_size_{0};
  uint32_t pin_coordinates_bytes_size_{0};
  std::atomic<bool> connection_closed_{false};

  std::vector<T_DjiWaypointV2> waypoints_;

  static void check_sigint();
  static void sigint_handler(int);
  static volatile sig_atomic_t sigint_received_;
};

#endif  //  DRONE_H_
