#ifndef SERVER_H_
#define SERVER_H_

#include <memory>  // std::unique_ptr

using T_DjiMopChannelHandle = void*;

/// \brief An exception is thrown when the communication channel is broken and
///     needs to be recreated
class pipeline_closed : public std::exception {};

/// \brief Server for communication with the drone control Android application
class server {
 public:
  server(uint16_t channel_id);
  ~server();

  /// \cond private
  server(const server&) = delete;
  server(server&&) = delete;

  server& operator=(const server&) = delete;
  server& operator=(server&&) = delete;
  /// \endcond

  T_DjiMopChannelHandle handle() const;

 private:
  T_DjiMopChannelHandle out_channel_handle_{nullptr};
};

#endif  // SERVER_H_
