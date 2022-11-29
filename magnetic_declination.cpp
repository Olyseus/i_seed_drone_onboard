#include "magnetic_declination.h"

#include <nlohmann/json.hpp>

magnetic_declination::magnetic_declination(const std::string& filename, double lat, double lon) {
  // FIXME (implement)
  (void)filename;
  (void)lat;
  (void)lon;
}

magnetic_declination::~magnetic_declination() = default;

auto magnetic_declination::declination() const -> double {
  return declination_;
}
