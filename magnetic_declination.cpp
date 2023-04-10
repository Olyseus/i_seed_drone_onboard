#include "magnetic_declination.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY
#include <fstream>           // std::ifstream
#include <nlohmann/json.hpp>

#include "json_sax_event_consumer.h"
#include "timer.h"

magnetic_declination::magnetic_declination(const boost::gregorian::date& date,
                                           const std::string& filename,
                                           double lat, double lon) {
  spdlog::info("Drone coordinates lat: {}, lon: {}", lat, lon);
  spdlog::info("Read magnetic declination table from {}", filename);

  std::ifstream f(filename);
  json_sax_event_consumer s(date, lat, lon);
  timer t;
  t.start();
  const bool ok{nlohmann::json::sax_parse(f, &s)};
  BOOST_VERIFY(ok);
  spdlog::info("Parsed in {}ms", t.elapsed_ms());

  declination_ = s.declination();
}

magnetic_declination::~magnetic_declination() = default;

auto magnetic_declination::declination() const -> double {
  return declination_;
}
