#ifndef CAMERA_PSDK_H_
#define CAMERA_PSDK_H_

#include <list>
#include <memory>  // std::unique_ptr
#include <mutex>
#include <regex>
#include <set>
#include <vector>

#include "inference.h"
#include "timer.h"

struct gps_coordinates {
  double longitude;
  double latitude;
  float altitude;
};

struct attitude {
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
  attitude drone_attitude;
  attitude gimbal_attitude;
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

  void shoot_photo(const gps_coordinates&, const attitude& drone_attitude,
      const attitude& gimbal_attitude);

  bool check_sdcard();

 private:
  inference inference_;

  std::mutex queue_mutex_;
  struct queue_entry {
    gps_coordinates gps;
    attitude drone_attitude;
    attitude gimbal_attitude;
  };
  std::list<queue_entry> queue_;
  timer file_waiting_timer_;

  std::mutex api_call_mutex_;

  std::vector<detection_result> detections_;

  std::set<uint32_t> already_present_indexes_;

  static const char* iso_name(int value);
  static const char* aperture_name(int value);
  static const char* shutter_speed_name(int value);
  static const char* compensation_name(int value);

  const std::regex jpeg_regex{".*_ZOOM\\.jpg"};
};

#endif // CAMERA_PSDK_H_
