#ifndef LASER_RANGE_H_
#define LASER_RANGE_H_

#include <random> // std::random_device

class laser_range {
 public:
  double latest();

 private:
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  std::random_device dev_;
  std::uniform_real_distribution<double> dist_{15.0, 17.0};
#endif
};

#endif // LASER_RANGE_H_
