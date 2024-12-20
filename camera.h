#ifndef CAMERA_H_
#define CAMERA_H_

#include <dji_camera_manager.h>  // T_DjiCameraManagerFileCreateTime

#include <list>
#include <mutex>
#include <regex>
#include <set>

#include "detection_result.h"
#include "inference.h"
#include "timer.h"

class mission;

// clang-format off
/// \brief Wrapper for a
///     <a href="https://developer.dji.com/doc/payload-sdk-api-reference/en/module/camera-manager.html">camera management</a>
///     Payload SDK functions
// clang-format on
class camera {
 public:
  camera(const std::string& model_file, mission&);
  ~camera();

  /// \cond private
  camera(const camera&) = delete;
  camera(camera&&) = delete;

  camera& operator=(const camera&) = delete;
  camera& operator=(camera&&) = delete;
  /// \endcond

  /// \brief Start shooting a photo
  /// \param[in] gps Drone's GPS coordinates
  /// \param[in] drone_attitude Drone's yaw/roll/pitch
  /// \param[in] gimbal_attitude Camera gimbal's yaw/roll/pitch
  /// \param[in] waypoint_index Waypoint index where result should be saved
  /// \note \ref thread_action "Thread: action"
  void shoot_photo(const gps_coordinates& gps, const attitude& drone_attitude,
                   const attitude& gimbal_attitude, std::size_t waypoint_index);

  /// \brief Check SDCard for new shot photos
  /// \return \b true New photos were found, and the inference was run
  /// \return \b false No new files found yet
  /// \note \ref thread_inference "Thread: inference"
  bool check_sdcard(bool debug_launch);

  /// \brief Check if processing queue is empty
  /// \return \b true There are no files left to process
  bool queue_is_empty() const;

 private:
  void process_inference_files(
      const std::vector<std::pair<uint32_t, T_DjiCameraManagerFileCreateTime>>&
          inference_files,
      bool debug_launch);

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

#endif  // CAMERA_H_
