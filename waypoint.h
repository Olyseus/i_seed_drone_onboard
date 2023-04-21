#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include <optional>

#include "detection_result.h"

/// \brief Waypoint data
class waypoint {
 public:
  waypoint(double latitude, double longitude);
  ~waypoint();

  waypoint(const waypoint&);
  waypoint(waypoint&&) noexcept;

  /// \cond private
  waypoint& operator=(const waypoint&) = delete;
  waypoint& operator=(waypoint&&) = delete;
  /// \endcond

  /// \return true if the current waypoint's altitude has a default value
  bool is_default_altitude() const;

  /// \param[in] laser_range
  ///     Drone's height value returned from laser measurement
  void set_custom_altitude(float laser_range);

  /// \brief Waypoint altitude
  float altitude() const;

  /// \brief Waypoint latitude
  double lat() const { return latitude_; }

  /// \brief Waypoint longitude
  double lon() const { return longitude_; }

  /// \brief Waypoint expected height, default height
  static constexpr float expected_height{15.0F};  // 15m

  /// \brief Mark waypoint as ready
  /// \param[in] is_forward true - forward mission is ready, false - backward
  ///     mission is ready
  void set_ready(bool is_forward);

  /// \return true If forward mission is ready
  bool is_forward_ready() const;

  /// \return true If backward mission is ready
  bool is_backward_ready() const;

  /// \return true If some objects were detected
  bool has_detection() const;

  /// \return Expected drone's heading in this waypoint
  /// \note In the forward mission, the drone heading is automatic, but when
  ///     the backward mission is launched, the heading is set manually to
  ///     match the same drone's position as in the forward mission.
  float heading() const;

  /// \brief When inference finished, save the detected objects
  void save_detection(const detection_result&);

  /// \brief Retrieve the detection results of the waypoint
  detection_result get_detection() const;

 private:
  const double latitude_;
  const double longitude_;
  std::optional<float> mission_altitude_;
  bool is_forward_ready_{false};
  bool is_backward_ready_{false};
  std::optional<detection_result> detection_result_;
};

#endif  // WAYPOINT_H_
