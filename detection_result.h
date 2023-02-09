#ifndef DETECTION_RESULT_H_
#define DETECTION_RESULT_H_

#include <vector>

struct gps_coordinates {
  double longitude;
  double latitude;
  float altitude;
  float relative_altitude;
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

#endif // DETECTION_RESULT_H_
