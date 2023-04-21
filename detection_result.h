#ifndef DETECTION_RESULT_H_
#define DETECTION_RESULT_H_

#include <vector>

/// \brief Drone GPS coordinates
/// \note See \ref drone for field sizes notes
struct gps_coordinates {
  double longitude;
  double latitude;
  float altitude;
  // FIXME (???) float relative_altitude;
};

/// \brief Drone and gimbal attitude
/// \note See \ref drone for field sizes notes
struct attitude {
  float pitch;
  float roll;
  float yaw;
};

/// \brief 2D coordinates of detection
struct detected_pixel {
  float x;
  float y;
};

/// \brief Full information about detected object
struct detection_result {
  gps_coordinates gps;
  attitude drone_attitude;
  attitude gimbal_attitude;
  std::vector<detected_pixel> pixels;
};

#endif  // DETECTION_RESULT_H_
