#ifndef SERVER_H_
#define SERVER_H_

#include <memory>  // std::unique_ptr

class pipeline_closed {};

class server {
 public:
  server(uint16_t channel_id);
  ~server();

  server(const server&) = delete;
  server(server&&) = delete;

  server& operator=(const server&) = delete;
  server& operator=(server&&) = delete;

  T_PsdkMopChannelHandle handle() const { return out_channel_handle_; }

 private:
  T_PsdkMopChannelHandle out_channel_handle_{nullptr};
};

#endif  // SERVER_H_
