#ifndef LASER_RANGE_H_
#define LASER_RANGE_H_

#include <random> // std::random_device

class laser_range {
 public:
  laser_range();

  double latest();

 private:
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  std::vector<double> values_;
#endif
};

#endif // LASER_RANGE_H_
