#ifndef DRONE_H_
#define DRONE_H_

// Onboard SDK
#include <osdk_typedef.h>  // E_OsdkStat
#include <signal.h>        // sig_atomic_t

#include <atomic>
#include <cstdint>  // uint16_t
#include <list>
#include <mutex>

#include "interconnection.pb.h"
#include "mission_state.h"

class LinuxSetup;

struct _cmdInfo;
typedef struct _cmdInfo T_CmdInfo;

struct _CommandHandle;
typedef struct _CommandHandle T_CmdHandle;

namespace DJI {
namespace OSDK {
class MopPipeline;
class Vehicle;
class WaypointV2;
}  // namespace OSDK
}  // namespace DJI

class drone {
 public:
  drone(int argc, char** argv);
  ~drone();

  drone(const drone&) = delete;
  drone(drone&&) = delete;

  drone& operator=(const drone&) = delete;
  drone& operator=(drone&&) = delete;

  void start();

 private:
  void receive_data_job();
  void send_data_job();
  void send_command(interconnection::command_type::command_t);
  void receive_data(std::string* buffer);
  void send_data(std::string& buffer);
  DJI::OSDK::WaypointV2 make_waypoint(double latitude, double longitude,
                                      float relative_height);
  static E_OsdkStat update_mission_state(T_CmdHandle* cmd_handle,
                                         const T_CmdInfo* cmd_info,
                                         const uint8_t* cmd_data,
                                         void* user_data);
  static E_OsdkStat update_mission_event(T_CmdHandle* cmd_handle,
                                         const T_CmdInfo* cmd_info,
                                         const uint8_t* cmd_data,
                                         void* user_data);

  static constexpr uint16_t channel_id{
      9745};  // Just a random number. Keep it consistent with Mobile SDK
  static constexpr int32_t protocol_version{
      6};  // Keep it consistent with Mobile SDK
  static constexpr int pkg_index{0};
  static constexpr int timeout{10};

  std::unique_ptr<LinuxSetup> linux_setup_;
  DJI::OSDK::Vehicle* vehicle_{nullptr};
  DJI::OSDK::MopPipeline* pipeline_{nullptr};

  mission_state mission_state_;

  uint32_t command_bytes_size_{0};
  uint32_t pin_coordinates_bytes_size_{0};
  std::atomic<bool> connection_closed_{false};
  std::mutex m_;
  std::list<interconnection::command_type::command_t> execute_commands_;

  static void check_sigint();
  static void sigint_handler(int);
  static volatile sig_atomic_t sigint_received_;
};

#endif  //  DRONE_H_
