#ifndef SERVER_H_
#define SERVER_H_

#include <memory>  // std::unique_ptr

using T_DjiMopChannelHandle = void*;

class pipeline_closed {};

class server {
 public:
  server(uint16_t channel_id);
  ~server();

  server(const server&) = delete;
  server(server&&) = delete;

  server& operator=(const server&) = delete;
  server& operator=(server&&) = delete;

  T_DjiMopChannelHandle handle() const;

 private:
  T_DjiMopChannelHandle out_channel_handle_{nullptr};
};

#endif  // SERVER_H_
