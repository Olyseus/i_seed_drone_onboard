#ifndef HOME_ALTITUDE_H_
#define HOME_ALTITUDE_H_

#include <optional>
#include <vector>

/// \brief Altitude of the home point
class home_altitude {
 public:
  home_altitude();
  ~home_altitude() = default;

  /// \cond private
  home_altitude(const home_altitude&) = delete;
  home_altitude(home_altitude&&) = delete;

  home_altitude& operator=(const home_altitude&) = delete;
  home_altitude& operator=(home_altitude&&) = delete;
  /// \endcond

  /// \brief Mission started, ready for collecting data
  void mission_start();

  /// \brief Save the home altitude and check the consistency
  ///     of measurements
  void set_altitude(float drone_altitude, float mission_altitude,
                    float home_altitude);

  /// \return Get the saved value of home altitude
  float get_home_altitude() const;

  /// \brief Mission stopped, no more data expected
  void mission_stop();

 private:
  bool in_progress_{false};
  std::optional<float> home_altitude_;
};

#endif  // HOME_ALTITUDE_H_
