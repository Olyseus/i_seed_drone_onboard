#ifndef JSON_SAX_EVENT_CONSUMER_H_
#define JSON_SAX_EVENT_CONSUMER_H_

#include <boost/date_time/gregorian/gregorian.hpp>
#include <nlohmann/json.hpp>

class json_sax_event_consumer : public nlohmann::json::json_sax_t {
 public:
  json_sax_event_consumer(const boost::gregorian::date& date, double lat, double lon);
  ~json_sax_event_consumer();

  json_sax_event_consumer(const json_sax_event_consumer&) = delete;
  json_sax_event_consumer(json_sax_event_consumer&&) = delete;

  json_sax_event_consumer& operator=(const json_sax_event_consumer&) = delete;
  json_sax_event_consumer& operator=(json_sax_event_consumer&&) = delete;

  bool null() override;
  bool boolean(bool val) override;
  bool number_integer(number_integer_t val) override;
  bool number_unsigned(number_unsigned_t val) override;
  bool number_float(number_float_t val, const string_t& s) override;
  bool string(string_t& val) override;
  bool start_object(std::size_t elements) override;
  bool end_object() override;
  bool start_array(std::size_t elements) override;
  bool end_array() override;
  bool key(string_t& val) override;
  bool binary(nlohmann::json::binary_t& val) override;
  bool parse_error(std::size_t position, const std::string& last_token, const nlohmann::json::exception& ex) override;

  double declination() const;

 private:
  static double date_to_double(const boost::gregorian::date& date);

  enum class state {
    wait_for_result_key,
    result_key_found,
    reading_result_array,
    reading_cell,
    reading_date,
    reading_declination,
    reading_latitude,
    reading_longitude,
    reading_other,
    finished
  };

  struct entry {
    std::optional<double> date;
    std::optional<double> declination;
    std::optional<double> latitude;
    std::optional<double> longitude;
    bool warning{false};
  };

  const double date_;
  const double lat_;
  const double lon_;

  std::optional<entry> best_;
  std::optional<entry> current_;

  state state_{state::wait_for_result_key};
};

#endif  // JSON_SAX_EVENT_CONSUMER_H_
