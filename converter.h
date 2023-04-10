#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <Eigen/Geometry>  // Eigen::Vector3d
#include <GeographicLib/LocalCartesian.hpp>

#include "detection_result.h"  // gps_coordinates

struct converter_result {
  Eigen::Vector3d p;  // start of the ray
  Eigen::Vector3d v;  // direction of the ray (with length)
  Eigen::Vector3d d;  // down direction
};

class converter {
 public:
  // Gimbal yaw is relative to drone
  static converter_result run(const gps_coordinates&,
                              const attitude& drone_attitude,
                              const attitude& gimbal_attitude, double length);

 private:
  friend class converter_test;

  static Eigen::Vector3d local_ned_to_ecef(const GeographicLib::LocalCartesian&,
                                           const Eigen::Vector3d& p);
  static Eigen::Vector3d camera_to_drone_ned(const attitude& gimbal_attitude,
                                             double length);
  static Eigen::Vector3d drone_ned_to_local_ned(const attitude& drone_attitude,
                                                const Eigen::Vector3d& v);
};

#endif  // CONVERTER_H_
