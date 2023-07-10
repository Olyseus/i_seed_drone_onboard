#ifndef LAT_LON_H_
#define LAT_LON_H_

/// \brief Latitude/longitude container
class lat_lon {
 public:
  lat_lon(double latitude, double longitude)
      : latitude_(latitude), longitude_(longitude) {}
  ~lat_lon() = default;

  double latitude() const { return latitude_; }
  double longitude() const { return longitude_; }

 private:
  double latitude_;
  double longitude_;
};

#endif  // LAT_LON_H_
