#ifndef HOME_ALTITUDE_H_
#define HOME_ALTITUDE_H_

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

  void set_altitude(double drone_altitude, double mission_altitude);
  double get_home_altitude() const;

  void mission_stop();

 private:
  double average_drone_altitude() const;

  bool in_progress_{false};
  std::vector<double> altitudes_;
  double mission_altitude_{0.0};
};

#endif // HOME_ALTITUDE_H_
