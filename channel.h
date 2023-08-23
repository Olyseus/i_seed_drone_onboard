#ifndef CHANNEL_H_
#define CHANNEL_H_

class server;

using T_DjiMopChannelHandle = void*;

class channel {
 public:
  explicit channel(const server&);
  ~channel();

  /// \cond private
  channel(const channel&) = delete;
  channel(channel&&) = delete;

  channel& operator=(const channel&) = delete;
  channel& operator=(channel&&) = delete;
  /// \endcond

  T_DjiMopChannelHandle out_handle() const;

 private:
  T_DjiMopChannelHandle out_channel_handle_{nullptr};
};

#endif  // CHANNEL_H_
