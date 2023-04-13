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

  void set_altitude(float drone_altitude, float mission_altitude,
                    float home_altitude);
  float get_home_altitude() const;

  void mission_stop();

 private:
  bool in_progress_{false};
  std::optional<float> home_altitude_;
};

#endif  // HOME_ALTITUDE_H_
