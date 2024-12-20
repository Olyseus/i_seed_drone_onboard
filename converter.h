#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <Eigen/Geometry>  // Eigen::Vector3d
#include <GeographicLib/LocalCartesian.hpp>

#include "detection_result.h"  // gps_coordinates

/// \brief ECEF ray based on laser range measurement and drone's coordinates
struct converter_result {
  /// \brief Start of the ray
  Eigen::Vector3d p;

  /// \brief Direction of the ray (with lenght)
  Eigen::Vector3d v;

  /// \brief Drone's down direction
  Eigen::Vector3d d;
};

/// \brief Conversion from geodetic lat/lon to geocentric x/y/z
class converter {
 public:
  /// \brief Run conversion
  /// \param[in] gps Drone's GPS coordinates
  /// \param[in] drone_attitude Drone's roll/pitch/yaw
  /// \param[in] gimbal_attitude Camera gimbal's roll/pitch/yaw.
  ///     Yaw is relative to drone
  /// \param[in] length The length of ray from camera.
  ///     The value of laser range measurement
  /// \return \ref converter_result
  static converter_result run(const gps_coordinates& gps,
                              const attitude& drone_attitude,
                              const attitude& gimbal_attitude, float length);

  // \brief ECEF coordinates of object by pointing vector and length
  /// \param[in] gps Drone's GPS coordinates
  /// \param[in] drone_attitude Drone's roll/pitch/yaw
  /// \param[in] pointing_vec Vector in drone's NED coordinates
  ///     It's a vector from detector to object, i.e. if object is
  ///     perfectly down, then the pointing_vec is (0, 0, v)
  ///     The lenght of the vector will not be used
  ///     Hint: Z expected to be positive (down)
  /// \param[in] length The length of ray from camera.
  ///     The value of laser range measurement
  /// \return Global ECEF (X,Y,Z)
  static Eigen::Vector3d get_ecef(const gps_coordinates& gps,
                                  const attitude& drone_attitude,
                                  const Eigen::Vector3d& pointing_vec,
                                  float length);

 private:
  /// \cond private
  friend class converter_test;
  /// \endcond

  static Eigen::Vector3d local_ned_to_ecef(const GeographicLib::LocalCartesian&,
                                           const Eigen::Vector3d& p);
  static Eigen::Vector3d camera_to_drone_ned(const attitude& gimbal_attitude,
                                             float length);
  static Eigen::Vector3d drone_ned_to_local_ned(const attitude& drone_attitude,
                                                const Eigen::Vector3d& v);

  static Eigen::Vector3d drone_ned_p();
};

#endif  // CONVERTER_H_
