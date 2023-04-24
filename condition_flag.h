#ifndef CONDITION_FLAG_H_
#define CONDITION_FLAG_H_

#include <condition_variable>
#include <mutex>

/// \brief Wrapper for a
///     <a href="https://en.cppreference.com/w/cpp/thread/condition_variable">std::condition_variable</a>
class condition_flag {
 public:
  condition_flag() noexcept;
  ~condition_flag();

  /// \cond private
  condition_flag(const condition_flag&) = delete;
  condition_flag(condition_flag&&) = delete;
  condition_flag& operator=(const condition_flag&) = delete;
  condition_flag& operator=(condition_flag&&) = delete;
  /// \endcond

  /// \brief Wait for a notification
  /// \note Thread: action
  void wait();

  /// \brief Notify waiting thread
  /// \note Thread: Payload SDK callback, main (on error)
  void notify();

 private:
  std::mutex m_;
  std::condition_variable condition_;
  bool flag_{false};
};

#endif  // CONDITION_FLAG_H_
