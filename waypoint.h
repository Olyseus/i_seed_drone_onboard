#ifndef WAYPOINT_H_
#define WAYPOINT_H_

#include <optional>

class waypoint {
 public:
  waypoint(double latitude, double longitude);
  ~waypoint();

  waypoint(const waypoint&);
  waypoint(waypoint&&);

  waypoint& operator=(const waypoint&) = delete;
  waypoint& operator=(waypoint&&) = delete;

  bool is_default_altitude() const;
  void set_custom_altitude(double laser_range);
  double altitude() const;

  double lat() const { return latitude_; }
  double lon() const { return longitude_; }

  static constexpr double expected_height{15.0}; // 15m

  // Forward pass is ready
  void set_ready();
  bool is_ready() const;

 private:
  const double latitude_;
  const double longitude_;
  std::optional<double> mission_altitude_;
  bool is_ready_{false};
};

#endif // WAYPOINT_H_
