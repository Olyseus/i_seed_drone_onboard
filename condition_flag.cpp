#include "condition_flag.h"

#include "olyseus_verify.h"  // OLYSEUS_VERIFY

condition_flag::condition_flag() noexcept = default;
condition_flag::~condition_flag() = default;

void condition_flag::wait() {
  std::unique_lock lock{m_};
  condition_.wait(lock, [this] { return flag_; });
  OLYSEUS_VERIFY(flag_);

  // unset the flag and release lock
  // (we should avoid locks in Payload SDK callback)
  flag_ = false;
}

void condition_flag::notify() {
  {
    const std::lock_guard lock{m_};
    OLYSEUS_VERIFY(!flag_);
    flag_ = true;
  }
  condition_.notify_one();
}
