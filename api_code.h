#ifndef API_CODE_H_
#define API_CODE_H_

class api_code {
 public:
  template <class T>
  explicit api_code(const T&);

  bool success() const { return code_ == code::success; }
  bool retry() const { return code_ == code::retry; }

 private:
  void make_retry();

  enum class code { success, failure, retry };

  code code_{code::failure};
};

#endif  // API_CODE_H_
