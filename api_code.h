#ifndef API_CODE_H_
#define API_CODE_H_

// Payload SDK
#include <dji_error.h> // DjiErrorCode

class api_code {
 public:
  explicit api_code(const DjiErrorCode code);

  bool success() const { return code_ == code::success; }
  bool retry() const { return code_ == code::retry; }

 private:
  void make_retry();

  enum class code { success, failure, retry };

  code code_{code::failure};
};

#endif  // API_CODE_H_
