#ifndef DRONE_H_
#define DRONE_H_

#include <osdk_typedef.h>  // E_OsdkStat

#include <atomic>
#include <cstdint>  // uint16_t
#include <list>
#include <mutex>

#include "interconnection.pb.h"

class LinuxSetup;

struct _cmdInfo;
typedef struct _cmdInfo T_CmdInfo;

struct _CommandHandle;
typedef struct _CommandHandle T_CmdHandle;

namespace DJI {
namespace OSDK {
class MopPipeline;
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
  void read_job();
  void write_job();
  bool send_command(interconnection::command_type::command_t);
  bool read_data(std::string* buffer);
  DJI::OSDK::WaypointV2 make_waypoint(double latitude, double longitude,
                                      float relative_height);
  void mission_finished();

  static E_OsdkStat update_mission_state(T_CmdHandle* cmd_handle,
                                         const T_CmdInfo* cmd_info,
                                         const uint8_t* cmd_data,
                                         void* user_data);

  static constexpr uint16_t channel_id{
      9745};  // Just a random number. Keep it consistent with Mobile SDK
  static constexpr int32_t protocol_version{
      6};  // Keep it consistent with Mobile SDK
  static constexpr int pkg_index{0};

  std::unique_ptr<LinuxSetup> linux_setup_;

  DJI::OSDK::MopPipeline* pipeline_{nullptr};

  uint32_t command_bytes_size_{0};
  uint32_t pin_coordinates_bytes_size_{0};
  bool connection_closed_{false};
  std::atomic<bool> mission_is_started_{false};
  std::mutex m_;
  std::list<interconnection::command_type::command_t> execute_commands_;
};

#endif  //  DRONE_H_
