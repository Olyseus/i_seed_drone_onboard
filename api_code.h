#ifndef API_CODE_H_
#define API_CODE_H_

#include <dji_typedef.h>  // T_DjiReturnCode

class api_code {
 public:
  explicit api_code(T_DjiReturnCode code);

  bool success() const { return code_ == code::success; }
  bool retry() const { return code_ == code::retry; }

 private:
  void make_retry();

  enum class code { success, failure, retry };

  code code_{code::failure};
};

#endif  // API_CODE_H_
