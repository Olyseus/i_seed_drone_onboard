#ifndef TIMER_H_
#define TIMER_H_

#include <boost/assert.hpp>  // BOOST_VERIFY
#include <chrono>

/// \brief Simple timer
class timer {
 public:
  /// \brief Start timer
  void start() { start_ = clock::now(); }

  /// \brief Show elapsed time in milliseconds
  int64_t elapsed_ms() const {
    namespace ch = std::chrono;
    BOOST_VERIFY(start_ != time_point::min());
    const auto elapsed{clock::now() - start_};
    return ch::duration_cast<ch::milliseconds>(elapsed).count();
  }

 private:
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;

  time_point start_{time_point::min()};
};

#endif  // TIMER_H_
