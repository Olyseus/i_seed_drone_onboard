#ifndef LASER_RANGE_H_
#define LASER_RANGE_H_

class laser_range {
 public:
  double latest() const;

 private:
#if defined(I_SEED_DRONE_ONBOARD_SIMULATOR)
  std::random_device dev_;
  std::uniform_real_distribution<double> dist_(15.0, 17.0);
#endif
};

#endif // LASER_RANGE_H_
