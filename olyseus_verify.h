#ifndef OLYSEUS_VERIFY_H_
#define OLYSEUS_VERIFY_H_

#include <cstdint>  // uint64_t

// https://github.com/boostorg/assert/blob/boost-1.81.0/include/boost/assert.hpp#L44-L54

#include <boost/config.hpp>            // BOOST_LIKELY
#include <boost/current_function.hpp>  // BOOST_CURRENT_FUNCTION

namespace olyseus {

[[noreturn]] void assertion_failed(const char* expr, const char* function,
                                   const char* file, uint64_t line);

[[noreturn]] void assertion_failed_msg(const char* expr, const char* msg,
                                       const char* function, const char* file,
                                       uint64_t line);

}  // namespace olyseus

#define OLYSEUS_VERIFY(expr)                                                  \
  (BOOST_LIKELY(!!(expr))                                                     \
       ? ((void)0)                                                            \
       : ::olyseus::assertion_failed(#expr, BOOST_CURRENT_FUNCTION, __FILE__, \
                                     __LINE__))

#define OLYSEUS_VERIFY_MSG(expr, msg)                                        \
  (BOOST_LIKELY(!!(expr))                                                    \
       ? ((void)0)                                                           \
       : ::olyseus::assertion_failed_msg(#expr, msg, BOOST_CURRENT_FUNCTION, \
                                         __FILE__, __LINE__))

#define OLYSEUS_UNREACHABLE \
  OLYSEUS_VERIFY_MSG(false, "Unreachable code reached")

#endif  // OLYSEUS_VERIFY_H_
