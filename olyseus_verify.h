#ifndef OLYSEUS_VERIFY_H_
#define OLYSEUS_VERIFY_H_

#include <cstdint>  // uint64_t

// https://github.com/boostorg/assert/blob/boost-1.81.0/include/boost/assert.hpp#L44-L54

#include <spdlog/spdlog.h>

#include <boost/config.hpp>            // BOOST_LIKELY
#include <boost/current_function.hpp>  // BOOST_CURRENT_FUNCTION

namespace olyseus {

[[noreturn]] inline void assertion_failed_msg(const char* expr, const char* msg,
                                              const char* function,
                                              const char* file, uint64_t line) {
  // FIXME (use https://en.cppreference.com/w/cpp/utility/source_location)
  spdlog::critical("File: {}:{}, function: {}, condition: {}", file, line,
                   function, expr);
  throw std::runtime_error(msg);
}

[[noreturn]] inline void assertion_failed(const char* expr,
                                          const char* function,
                                          const char* file, uint64_t line) {
  assertion_failed_msg(expr, "Assertion failed", function, file, line);
}

template <class T_1, class T_2>
[[noreturn]] inline void assertion_failed_eq(T_1 val_1, T_2 val_2,
                                             const char* expr_1,
                                             const char* expr_2,
                                             const char* function,
                                             const char* file, uint64_t line) {
  // FIXME (use https://en.cppreference.com/w/cpp/utility/source_location)
  spdlog::critical(
      "\n"
      "    '{}' evaluates to {}\n"
      "    '{}' evaluates to {}\n"
      "File: {}:{}, function: {}",
      expr_1, val_1, expr_2, val_2, file, line, function);

  throw std::runtime_error("Values are not equal");
}

}  // namespace olyseus

#define OLYSEUS_VERIFY(expr)                                                  \
  (BOOST_LIKELY(static_cast<bool>(expr))                                      \
       ? ((void)0)                                                            \
       : ::olyseus::assertion_failed(#expr, BOOST_CURRENT_FUNCTION, __FILE__, \
                                     __LINE__))

#define OLYSEUS_VERIFY_MSG(expr, msg)                                        \
  (BOOST_LIKELY(static_cast<bool>(expr))                                     \
       ? ((void)0)                                                           \
       : ::olyseus::assertion_failed_msg(#expr, msg, BOOST_CURRENT_FUNCTION, \
                                         __FILE__, __LINE__))

#define OLYSEUS_VERIFY_EQ(val_1, val_2)                                   \
  (BOOST_LIKELY((val_1) == (val_2))                                       \
       ? ((void)0)                                                        \
       : ::olyseus::assertion_failed_eq(val_1, val_2, #val_1, #val_2,     \
                                        BOOST_CURRENT_FUNCTION, __FILE__, \
                                        __LINE__))
#define OLYSEUS_UNREACHABLE \
  OLYSEUS_VERIFY_MSG(false, "Unreachable code reached")

#endif  // OLYSEUS_VERIFY_H_
