#include <spdlog/spdlog.h>

#include <exception>  // std::runtime_error

/// \cond private
namespace boost {

void assertion_failed(const char* expr, const char* function, const char* file,
                      long line) {  // NOLINT(google-runtime-int)
  spdlog::critical("File: {}:{}, function: {}, condition: {}", file, line,
                   function, expr);
  throw std::runtime_error("assertion failed");
}

void assertion_failed_msg(const char* expr, const char* msg,
                          const char* function, const char* file,
                          long line) {  // NOLINT(google-runtime-int)
  spdlog::critical("File: {}:{}, function: {}, condition: {}", file, line,
                   function, expr);
  throw std::runtime_error(msg);
}

}  // namespace boost
/// \endcond
