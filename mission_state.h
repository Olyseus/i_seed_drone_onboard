#ifndef MISSION_STATE_H_
#define MISSION_STATE_H_

#include <mutex>

namespace DJI {
namespace OSDK {
class MissionStatePushAck;
}  // namespace OSDK
}  // namespace DJI

class mission_state {
 public:
  void start();
  void finish();

  bool is_started() const;
  bool is_disconnected() const;

  void update(const DJI::OSDK::MissionStatePushAck*);

 private:
  const char* state_name() const;

  bool is_started_{false};
  bool initial_update_received_{false};
  int64_t state_{0};
  uint16_t waypoint_index_{0};

  // Unknown state, receiving while using simulator,
  // after receiving disconnected multiple times
  static constexpr int64_t mission_unknown_state_7_{7};

  mutable std::mutex m_;
};

#endif  // MISSION_STATE_H_
