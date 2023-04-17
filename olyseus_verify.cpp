#include "olyseus_verify.h"

#include <spdlog/spdlog.h>

namespace olyseus {

[[noreturn]] void assertion_failed(const char* expr, const char* function,
                                   const char* file, uint64_t line) {
  // FIXME (use https://en.cppreference.com/w/cpp/utility/source_location)
  spdlog::critical("File: {}:{}, function: {}, condition: {}", file, line,
                   function, expr);
  throw std::runtime_error("assertion failed");
}

[[noreturn]] void assertion_failed_msg(const char* expr, const char* msg,
                                       const char* function, const char* file,
                                       uint64_t line) {
  spdlog::critical("File: {}:{}, function: {}, condition: {}", file, line,
                   function, expr);
  throw std::runtime_error(msg);
}

}  // namespace olyseus
