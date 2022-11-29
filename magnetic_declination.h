#ifndef MAGNETIC_DECLINATION_H_
#define MAGNETIC_DECLINATION_H_

#include <string>

#include <boost/date_time/gregorian/gregorian.hpp>

class magnetic_declination {
 public:
  magnetic_declination(const boost::gregorian::date& date, const std::string& filename, double lat, double lon);
  ~magnetic_declination();

  double declination() const;

  magnetic_declination(const magnetic_declination&) = delete;
  magnetic_declination(magnetic_declination&&) = delete;

  magnetic_declination& operator=(const magnetic_declination&) = delete;
  magnetic_declination& operator=(magnetic_declination&&) = delete;

 private:
  double declination_{0.0};
};

#endif //  MAGNETIC_DECLINATION_H_
