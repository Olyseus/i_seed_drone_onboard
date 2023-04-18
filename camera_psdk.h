#ifndef CAMERA_PSDK_H_
#define CAMERA_PSDK_H_

#include <dji_camera_manager.h>  // T_DjiCameraManagerFileCreateTime

#include <list>
#include <memory>  // std::unique_ptr
#include <mutex>
#include <regex>
#include <set>
#include <vector>

#include "detection_result.h"
#include "inference.h"
#include "timer.h"

class mission;

class camera_psdk {
 public:
  camera_psdk(const std::string& model_file, mission&);
  ~camera_psdk();

  camera_psdk(const camera_psdk&) = delete;
  camera_psdk(camera_psdk&&) = delete;

  camera_psdk& operator=(const camera_psdk&) = delete;
  camera_psdk& operator=(camera_psdk&&) = delete;

  void shoot_photo(const gps_coordinates&, const attitude& drone_attitude,
                   const attitude& gimbal_attitude, std::size_t waypoint_index);

  bool check_sdcard();
  bool queue_is_empty() const;

 private:
  void process_inference_files(
      const std::vector<std::pair<uint32_t, T_DjiCameraManagerFileCreateTime>>&
          inference_files);

  inference inference_;
  mission& mission_;

  mutable std::mutex queue_mutex_;
  struct queue_entry {
    gps_coordinates gps;
    attitude drone_attitude;
    attitude gimbal_attitude;
    std::size_t waypoint_index;
  };
  std::list<queue_entry> queue_;
  timer file_waiting_timer_;

  std::mutex api_call_mutex_;

  std::set<uint32_t> already_present_indexes_;

  static const char* iso_name(int value);
  static const char* aperture_name(int value);
  static const char* shutter_speed_name(int value);
  static const char* compensation_name(int value);

  const std::regex jpeg_regex{".*_ZOOM\\.jpg"};
};

#endif  // CAMERA_PSDK_H_
