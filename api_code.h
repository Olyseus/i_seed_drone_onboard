#ifndef API_CODE_H_
#define API_CODE_H_

#include <dji_typedef.h>  // T_DjiReturnCode

/// \brief Wrapper for the
///     <a href="https://developer.dji.com/doc/payload-sdk-api-reference/en/core/dji-error.html">T_DjiReturnCode</a>
///     from Payload SDK
class api_code {
 public:
  /// \throw pipeline_closed
  explicit api_code(T_DjiReturnCode code);

  /// \return \b true If the API call succeeds
  bool success() const { return code_ == code::success; }

  /// \return \b true If the API call fails, and request should be repeated
  bool retry() const { return code_ == code::retry; }

 private:
  void make_retry();

  enum class code { success, failure, retry };

  code code_{code::failure};
};

#endif  // API_CODE_H_
