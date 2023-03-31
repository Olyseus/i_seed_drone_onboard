#ifndef CONDITION_FLAG_H_
#define CONDITION_FLAG_H_

#include <condition_variable>
#include <mutex>

class condition_flag {
 public:
  condition_flag();
  ~condition_flag();

  condition_flag(const condition_flag&) = delete;
  condition_flag(condition_flag&&) = delete;
  condition_flag& operator=(const condition_flag&) = delete;
  condition_flag& operator=(condition_flag&&) = delete;

  // thread: action
  void wait();

  // thread: Payload SDK callback, main (on error)
  void notify();

 private:
  std::mutex m_;
  std::condition_variable condition_;
  bool flag_{false};
};

#endif // CONDITION_FLAG_H_
