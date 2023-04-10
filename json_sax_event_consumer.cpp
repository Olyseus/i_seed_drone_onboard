#include "json_sax_event_consumer.h"

#include <spdlog/spdlog.h>

#include <boost/assert.hpp>  // BOOST_VERIFY

json_sax_event_consumer::json_sax_event_consumer(
    const boost::gregorian::date& date, double lat, double lon)
    : date_(date_to_double(date)), lat_(lat), lon_(lon) {
  spdlog::info("JSON parser for lat: {}, lon: {}, date (float): {}", lat_, lon_,
               date_);
}

json_sax_event_consumer::~json_sax_event_consumer() = default;

auto json_sax_event_consumer::null() -> bool {
  spdlog::error("null");
  return false;
}

auto json_sax_event_consumer::boolean(bool val) -> bool {
  spdlog::error("boolean: {}", val);
  return false;
}

auto json_sax_event_consumer::number_integer(number_integer_t val) -> bool {
  return json_sax_event_consumer::number_float(static_cast<double>(val), "");
}

auto json_sax_event_consumer::number_unsigned(number_unsigned_t val) -> bool {
  return json_sax_event_consumer::number_float(static_cast<double>(val), "");
}

auto json_sax_event_consumer::number_float(number_float_t val,
                                           const string_t& s) -> bool {
  switch (state_) {
    case state::reading_date:
      BOOST_VERIFY(current_.has_value());
      BOOST_VERIFY(!current_->date.has_value());
      current_->date = val;
      state_ = state::reading_cell;
      return true;
    case state::reading_declination:
      BOOST_VERIFY(current_.has_value());
      BOOST_VERIFY(!current_->declination.has_value());
      current_->declination = val;
      state_ = state::reading_cell;
      return true;
    case state::reading_latitude:
      BOOST_VERIFY(current_.has_value());
      BOOST_VERIFY(!current_->latitude.has_value());
      current_->latitude = val;
      state_ = state::reading_cell;
      return true;
    case state::reading_longitude:
      BOOST_VERIFY(current_.has_value());
      BOOST_VERIFY(!current_->longitude.has_value());
      current_->longitude = val;
      state_ = state::reading_cell;
      return true;
    case state::reading_other:
      BOOST_VERIFY(current_.has_value());
      state_ = state::reading_cell;
      return true;
    default:
      spdlog::error("number_float: {}, {}", val, s);
      return false;
  }
}

auto json_sax_event_consumer::string(string_t& val) -> bool {
  switch (state_) {
    case state::wait_for_result_key:
      BOOST_VERIFY(!current_.has_value());
      BOOST_VERIFY(!best_.has_value());
      return true;
    case state::reading_other:
      BOOST_VERIFY(current_.has_value());
      state_ = state::reading_cell;
      return true;
    case state::finished:
      BOOST_VERIFY(best_.has_value());
      BOOST_VERIFY(!current_.has_value());
      return true;
    default:
      spdlog::error("string: {}", val);
      return false;
  }
}

auto json_sax_event_consumer::start_object(std::size_t elements) -> bool {
  switch (state_) {
    case state::wait_for_result_key:
      BOOST_VERIFY(!current_.has_value());
      BOOST_VERIFY(!best_.has_value());
      return true;
    case state::reading_result_array:
      BOOST_VERIFY(!current_.has_value());
      state_ = state::reading_cell;
      return true;
    case state::finished:
      BOOST_VERIFY(best_.has_value());
      BOOST_VERIFY(!current_.has_value());
      return true;
    default:
      spdlog::error("start_object: {}", elements);
      return false;
  }
}

auto json_sax_event_consumer::end_object() -> bool {
  switch (state_) {
    case state::reading_cell:
      BOOST_VERIFY(current_.has_value());
      BOOST_VERIFY(current_->date.has_value());
      BOOST_VERIFY(current_->declination.has_value());
      BOOST_VERIFY(current_->latitude.has_value());
      BOOST_VERIFY(current_->longitude.has_value());
      if (best_.has_value()) {
        BOOST_VERIFY(best_->date.has_value());
        BOOST_VERIFY(best_->declination.has_value());
        BOOST_VERIFY(best_->latitude.has_value());
        BOOST_VERIFY(best_->longitude.has_value());
        if (std::abs(best_->date.value() - date_) >
            std::abs(current_->date.value() - date_)) {
          best_ = current_;
        } else if (std::abs(best_->latitude.value() - lat_) >
                   std::abs(current_->latitude.value() - lat_)) {
          best_ = current_;
        } else if (std::abs(best_->longitude.value() - lon_) >
                   std::abs(current_->longitude.value() - lon_)) {
          best_ = current_;
        }
      } else {
        best_ = current_;
      }
      current_.reset();
      BOOST_VERIFY(!current_.has_value());
      BOOST_VERIFY(best_.has_value());
      state_ = state::reading_result_array;
      return true;
    case state::finished:
      BOOST_VERIFY(best_.has_value());
      BOOST_VERIFY(!current_.has_value());
      return true;
    default:
      spdlog::error("end_object");
      return false;
  }
}

auto json_sax_event_consumer::start_array(std::size_t elements) -> bool {
  switch (state_) {
    case state::result_key_found:
      BOOST_VERIFY(!current_.has_value());
      BOOST_VERIFY(!best_.has_value());
      state_ = state::reading_result_array;
      return true;
    default:
      spdlog::info("start_array: {}", elements);
      return false;
  }
}

auto json_sax_event_consumer::end_array() -> bool {
  switch (state_) {
    case state::reading_result_array:
      BOOST_VERIFY(best_.has_value());
      BOOST_VERIFY(!current_.has_value());
      state_ = state::finished;
      if (best_->warning) {
        spdlog::warn("Unreliable magnetic declination");
      }
      spdlog::info("Best cell lat({}) lon({}) date({})",
                   best_->latitude.value(), best_->longitude.value(),
                   best_->date.value());
      return true;
    default:
      spdlog::info("end_array");
      return false;
  }
}

auto json_sax_event_consumer::key(string_t& val) -> bool {
  switch (state_) {
    case state::wait_for_result_key:
      BOOST_VERIFY(!current_.has_value());
      BOOST_VERIFY(!best_.has_value());
      if (val == "result") {
        state_ = state::result_key_found;
      }
      return true;
    case state::reading_cell:
      if (!current_.has_value()) {
        current_ = entry{};
      }
      BOOST_VERIFY(current_.has_value());
      if (val == "date") {
        state_ = state::reading_date;
      } else if (val == "declination") {
        state_ = state::reading_declination;
      } else if (val == "latitude") {
        state_ = state::reading_latitude;
      } else if (val == "longitude") {
        state_ = state::reading_longitude;
      } else if (val == "warning") {
        BOOST_VERIFY(!current_->warning);
        current_->warning = true;
        state_ = state::reading_other;
      } else {
        state_ = state::reading_other;
      }
      return true;
    case state::finished:
      BOOST_VERIFY(best_.has_value());
      BOOST_VERIFY(!current_.has_value());
      return true;
    default:
      spdlog::error("key: {}", val);
      return false;
  }
}

auto json_sax_event_consumer::binary(nlohmann::json::binary_t& val) -> bool {
  (void)val;
  spdlog::error("binary[...]");
  return false;
}

auto json_sax_event_consumer::parse_error(std::size_t position,
                                          const std::string& last_token,
                                          const nlohmann::json::exception& ex)
    -> bool {
  spdlog::error("parse_error: {}, {}, {}", position, last_token, ex.what());
  return false;
}

auto json_sax_event_consumer::declination() const -> double {
  BOOST_VERIFY(best_.has_value());
  BOOST_VERIFY(best_->declination.has_value());
  return best_->declination.value();
}

auto json_sax_event_consumer::date_to_double(const boost::gregorian::date& date)
    -> double {
  const boost::gregorian::partial_date jan_1{1, boost::gregorian::Jan};
  const long int number_of_days_in_year{
      (jan_1(date.year() + 1) - jan_1(date.year())).days()};
  BOOST_VERIFY(number_of_days_in_year <= 366);
  BOOST_VERIFY(number_of_days_in_year >= 365);

  const auto day_of_year{date.day_of_year()};
  BOOST_VERIFY(day_of_year >= 1);
  BOOST_VERIFY(day_of_year <= number_of_days_in_year);

  return date.year() + (day_of_year - 1.0) / number_of_days_in_year;
}
