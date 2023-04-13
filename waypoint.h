#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include <optional>

#include "detection_result.h"

class waypoint {
 public:
  waypoint(double latitude, double longitude);
  ~waypoint();

  waypoint(const waypoint&);
  waypoint(waypoint&&) noexcept;

  waypoint& operator=(const waypoint&) = delete;
  waypoint& operator=(waypoint&&) = delete;

  bool is_default_altitude() const;
  void set_custom_altitude(double laser_range);
  double altitude() const;

  double lat() const { return latitude_; }
  double lon() const { return longitude_; }

  static constexpr double expected_height{15.0};  // 15m

  // Forward pass is ready
  void set_ready(bool is_forward);
  bool is_forward_ready() const;
  bool is_backward_ready() const;
  bool has_detection() const;
  double heading() const;

  void save_detection(const detection_result&);
  detection_result get_detection() const;

 private:
  const double latitude_;
  const double longitude_;
  std::optional<double> mission_altitude_;
  bool is_forward_ready_{false};
  bool is_backward_ready_{false};
  std::optional<detection_result> detection_result_;
};

#endif  // WAYPOINT_H_
