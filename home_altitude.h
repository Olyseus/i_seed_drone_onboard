#ifndef HOME_ALTITUDE_H_
#define HOME_ALTITUDE_H_

#include <optional>
#include <vector>

class home_altitude {
 public:
  home_altitude();

  home_altitude(const home_altitude&) = delete;
  home_altitude(home_altitude&&) = delete;

  home_altitude& operator=(const home_altitude&) = delete;
  home_altitude& operator=(home_altitude&&) = delete;

  ~home_altitude();

  void mission_start();

  void set_altitude(double drone_altitude, double mission_altitude, double home_altitude);
  double get_home_altitude() const;

  void mission_stop();

 private:
  bool in_progress_{false};
  std::optional<double> home_altitude_;
  double mission_altitude_{0.0};
};

#endif // HOME_ALTITUDE_H_
