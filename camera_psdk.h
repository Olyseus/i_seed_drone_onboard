#ifndef CAMERA_PSDK_H_
#define CAMERA_PSDK_H_

#include <list>
#include <memory>  // std::unique_ptr
#include <mutex>
#include <regex>
#include <set>
#include <vector>

#include "inference.h"

struct gps_coordinates {
  double longitude;
  double latitude;
  float altitude;
};

struct quaternion {
  float q0;
  float q1;
  float q2;
  float q3;
};

struct gimbal_data {
  float pitch;
  float roll;
  float yaw;
};

struct detected_pixel {
  float x;
  float y;
};

struct detection_result {
  gps_coordinates gps;
  quaternion quat;
  gimbal_data gimbal;
  std::vector<detected_pixel> pixels;
};

class camera_psdk {
 public:
  camera_psdk(const std::string& model_file);
  ~camera_psdk();

  camera_psdk(const camera_psdk&) = delete;
  camera_psdk(camera_psdk&&) = delete;

  camera_psdk& operator=(const camera_psdk&) = delete;
  camera_psdk& operator=(camera_psdk&&) = delete;

  void shoot_photo(const gps_coordinates&, const quaternion&,
      const gimbal_data&);

  void check_sdcard();

 private:
  inference inference_;

  std::mutex queue_mutex_;
  struct queue_entry {
    gps_coordinates gps;
    quaternion quat;
    gimbal_data gimbal;
  };
  std::list<queue_entry> queue_;

  std::mutex api_call_mutex_;

  std::vector<detection_result> detections_;

  static constexpr int mount_position();
  std::set<uint32_t> already_present_indexes_;

  const std::regex jpeg_regex{".*_ZOOM\\.jpg"};
};

#endif // CAMERA_PSDK_H_
